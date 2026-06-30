import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import os
import copy
import collections
import random
import math
import xml.etree.ElementTree as ET

class RecurrentPolicyNetwork(nn.Module):
    """LSTM actor used to approximate recurrent MADDPG behavior."""

    def __init__(self, input_dim, out_dim, hidden_dim=64):
        super().__init__()
        self.hidden_dim = hidden_dim
        self.lstm = nn.LSTM(input_size=input_dim, hidden_size=hidden_dim, batch_first=True)
        self.fc1 = nn.Linear(hidden_dim, hidden_dim)
        self.fc3 = nn.Linear(hidden_dim, out_dim)

    def forward(self, x, hidden_state=None, return_hidden=False):
        if x.dim() == 1:
            x = x.unsqueeze(0).unsqueeze(0)
        elif x.dim() == 2:
            x = x.unsqueeze(1)

        if hidden_state is None:
            lstm_out, hidden_state = self.lstm(x)
        else:
            h_n, c_n = hidden_state
            h_n = h_n.to(x.device)
            c_n = c_n.to(x.device)
            lstm_out, hidden_state = self.lstm(x, (h_n, c_n))

        h = torch.relu(self.fc1(lstm_out[:, -1, :]))
        out = self.fc3(h)
        if return_hidden:
            h_n, c_n = hidden_state
            return out, (h_n.detach(), c_n.detach())
        return out



class RelativePositionCommunicationLayer(nn.Module):
    """Transformer-style communication layer with learnable relative-position bias."""

    def __init__(self, hidden_dim, nhead, relation_bucket_count, dropout=0.1, ff_multiplier=4):
        super().__init__()
        self.hidden_dim = int(hidden_dim)
        self.nhead = int(nhead)
        if self.hidden_dim % self.nhead != 0:
            raise ValueError(f"hidden_dim ({hidden_dim}) must be divisible by nhead ({nhead})")
        self.head_dim = self.hidden_dim // self.nhead

        self.q_proj = nn.Linear(self.hidden_dim, self.hidden_dim)
        self.k_proj = nn.Linear(self.hidden_dim, self.hidden_dim)
        self.v_proj = nn.Linear(self.hidden_dim, self.hidden_dim)
        self.out_proj = nn.Linear(self.hidden_dim, self.hidden_dim)

        self.rel_bias = nn.Embedding(int(relation_bucket_count), self.nhead)
        self.attn_dropout = nn.Dropout(float(dropout))

        self.norm1 = nn.LayerNorm(self.hidden_dim)
        self.norm2 = nn.LayerNorm(self.hidden_dim)
        self.ffn = nn.Sequential(
            nn.Linear(self.hidden_dim, self.hidden_dim * int(ff_multiplier)),
            nn.GELU(),
            nn.Dropout(float(dropout)),
            nn.Linear(self.hidden_dim * int(ff_multiplier), self.hidden_dim),
        )

    def forward(self, x, relation_index):
        # x: [batch, intersections, hidden]
        bsz, n_nodes, _ = x.shape

        q = self.q_proj(x).view(bsz, n_nodes, self.nhead, self.head_dim).transpose(1, 2)
        k = self.k_proj(x).view(bsz, n_nodes, self.nhead, self.head_dim).transpose(1, 2)
        v = self.v_proj(x).view(bsz, n_nodes, self.nhead, self.head_dim).transpose(1, 2)

        scores = torch.matmul(q, k.transpose(-2, -1)) / math.sqrt(self.head_dim)
        rel_bias = self.rel_bias(relation_index).permute(2, 0, 1).unsqueeze(0)
        scores = scores + rel_bias

        attn = torch.softmax(scores, dim=-1)
        attn = self.attn_dropout(attn)

        context = torch.matmul(attn, v)
        context = context.transpose(1, 2).contiguous().view(bsz, n_nodes, self.hidden_dim)
        x = self.norm1(x + self.out_proj(context))
        x = self.norm2(x + self.ffn(x))
        return x



class TCMQNetwork(nn.Module):
    """Paper-style shared TCM communication network for all intersections."""

    def __init__(
        self,
        input_dim,
        hidden_dim,
        output_dim,
        nhead=4,
        num_layers=2,
        dropout=0.1,
        ff_multiplier=4,
        relation_bucket_count=100,
    ):
        super().__init__()
        self.hidden_dim = int(hidden_dim)

        self.state_embedding = nn.Sequential(
            nn.Linear(int(input_dim), self.hidden_dim),
            nn.ReLU(),
            nn.Linear(self.hidden_dim, self.hidden_dim),
            nn.ReLU(),
        )

        self.layers = nn.ModuleList([
            RelativePositionCommunicationLayer(
                hidden_dim=self.hidden_dim,
                nhead=nhead,
                relation_bucket_count=relation_bucket_count,
                dropout=dropout,
                ff_multiplier=ff_multiplier,
            )
            for _ in range(int(num_layers))
        ])
        self.q_head = nn.Linear(self.hidden_dim, int(output_dim))

    def forward(self, states, relation_index):
        # states: [batch, intersections, state_dim]
        x = self.state_embedding(states)
        for layer in self.layers:
            x = layer(x, relation_index)
        return self.q_head(x)


class DeepNN(nn.Module):
    def __init__(self, input_dim, out_dim, hidden_dim=64, nonlin=nn.functional.relu):
        super().__init__()
        self.hidden_dim = hidden_dim
        self.device = None
        self.fc1 = nn.Linear(input_dim, hidden_dim)
        self.fc2 = nn.Linear(hidden_dim, hidden_dim)
        self.fc3 = nn.Linear(hidden_dim, out_dim)
        self.nonlin = nonlin

        # Persistent hidden state for recurrent networks (maintained across timesteps in rollouts)
        self.h_n = None
        self.c_n = None

    def forward(self, X, use_hidden_state=False):
        """
        Forward pass.

        Args:
            X: Input tensor, shape (batch_size, input_dim) or (batch_size, seq_len, input_dim)
            use_hidden_state: If True and recurrent, use persistent hidden state (for rollouts).
                             If False, use fresh hidden state (for replay buffer training).
        """
        # Remove sequence dimension if present for linear networks
        if X.dim() == 3:
            X = X.squeeze(1)
        h = self.nonlin(self.fc1(X))

        h = self.nonlin(self.fc2(h))
        out = self.fc3(h)
        return out