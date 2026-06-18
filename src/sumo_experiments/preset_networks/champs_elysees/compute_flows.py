import pandas as pd

columns = ["Identifiant arc",
           "Libelle",
           "Date et heure de comptage",
           "Débit horaire",
           "Taux d'occupation",
           "Etat trafic",
           "Identifiant noeud amont",
           "Libelle noeud amont",
           "Identifiant noeud aval",
           "Libelle noeud aval",
           "Etat arc",
           "Date debut dispo data",
           "Date fin dispo data",
           "geo_point_2d",
           "geo_shape",
           "coordonnees",]
data = pd.read_csv('traffic_data.csv', names=columns, index_col=False)

liste_noeuds = ['Champs-Tilsitt', 'Av_Champs_Elysees-Face_Air_Franc', 'Av_Champs_Elysees-Balzac',  'Av_Champs_Elysees-Washington', 'Av_Champs_Elysees-Berri', 'Av_Champs_Elysees-La_Boetie',  'Av_Champs_Elysees-Colisee', 'Rond_Point_Champs_Elysees']


liste_noeuds = list(reversed(liste_noeuds))
flows = {}
for i in range(len(liste_noeuds) - 1):
    selection = data[(data['Libelle noeud amont'].isin([liste_noeuds[i]])) & (data['Libelle noeud aval'].isin([liste_noeuds[i+1]])) & (data["Débit horaire"].notna()) & (data["Débit horaire"] > 0) & (data["Date et heure de comptage"].map(lambda t: t.split('T')[1]) == "16:00:00+00:00")]
    #print(selection["Débit horaire"])
    flows[(liste_noeuds[i], liste_noeuds[i+1])] = selection["Taux d'occupation"].mean()

print(flows)
