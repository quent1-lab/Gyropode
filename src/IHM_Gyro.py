import tkinter as tk
from tkinter import ttk
import pygatt
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Création de la fenêtre principale
root = tk.Tk()
root.title("Contrôle Bluetooth")

# Création d'une figure pour le graphique
fig, ax = plt.subplots()
ax.set_xlabel('Temps (s)')
ax.set_ylabel('Valeurs')
line, = ax.plot([], [], 'r-', animated=True)

# Fonction d'animation pour le graphique
def animate(i):
    line.set_data(x_data, y_data)

# Initialisation des données du graphique
x_data, y_data = [], []

# Fonction pour recevoir les données Bluetooth et les traiter
def receive_data(data):
    try:
        # Décodage des données
        decoded_data = data.decode("utf-8")
        parts = decoded_data.strip()[1:-1].split()
        nom = parts[0]
        vals = list(map(float, parts[1:]))

        # Affichage des données
        print("Nom:", nom)
        print("Valeurs:", vals)

        # Mise à jour du graphique
        x_data.append(len(x_data) + 1)
        for i in range(len(vals)):
            if len(y_data) <= i:
                y_data.append([])
            y_data[i].append(vals[i])

        # Tracé du graphique
        ax.relim()
        ax.autoscale_view()

    except Exception as e:
        print("Erreur lors du traitement des données:", e)

# Création d'un widget pour afficher les données reçues
data_label = tk.Label(root, text="Données reçues:")
data_label.pack()

# Connexion Bluetooth
try:
    adapter = pygatt.GATTToolBackend()
    adapter.start()
    device = adapter.connect('10:97:BD:D3:1F:82')
except Exception as e:
    print("Erreur lors de la connexion Bluetooth:", e)

# Création de sliders pour envoyer des valeurs
send_label = tk.Label(root, text="Envoyer des valeurs:")
send_label.pack()

sliders = []
for i in range(3):  # Nombre de valeurs à envoyer
    slider = tk.Scale(root, from_=0, to=100, orient="horizontal")
    slider.pack()
    sliders.append(slider)

# Fonction pour envoyer les valeurs via Bluetooth
def send_values():
    try:
        values = [slider.get() for slider in sliders]
        message = f"(my_robot {' '.join(map(str, values))})"
        device.char_write("0000ffe1-0000-1000-8000-00805f9b34fb", bytearray(message, "utf-8"))
        print("Données envoyées:", values)
    except Exception as e:
        print("Erreur lors de l'envoi des données:", e)

# Bouton pour envoyer les valeurs
send_button = tk.Button(root, text="Envoyer", command=send_values)
send_button.pack()

# Démarrage de l'animation du graphique
ani = FuncAnimation(fig, animate, interval=1000)

# Démarrage de la boucle principale
root.mainloop()
