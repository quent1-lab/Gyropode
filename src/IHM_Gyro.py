import tkinter as tk
import tkinter.ttk as ttk
import bluetooth
import threading

# Fonction pour envoyer des données via Bluetooth
def send_data():
    nom = nom_entry.get()
    val1 = val1_slider.get()
    val2 = val2_slider.get()
    val3 = val3_slider.get()
    message = f"({nom} {val1} {val2} {val3})"
    print("Envoi:", message)
    # Code pour envoyer le message via Bluetooth
    # Remplacer cette partie par votre propre logique d'envoi Bluetooth

# Fonction pour recevoir des données via Bluetooth
def receive_data():
    target_name = "Gyro_Q"
    target_address = None

    nearby_devices = bluetooth.discover_devices()

    for bdaddr in nearby_devices:
        if target_name == bluetooth.lookup_name(bdaddr):
            target_address = bdaddr
            break

    if target_address is not None:
        print(f"found target bluetooth device with address {target_address}")
        client_socket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
        client_socket.connect((target_address, 1))  # The second parameter is the port number
    else:
        print("could not find target bluetooth device nearby")

    while True:
        data = client_socket.recv(1024)
        if data:
            message = data.decode()
            print("Reçu:", message)
            # Code pour décoder le message et faire quelque chose avec les valeurs
            # Remplacer cette partie par votre propre logique de traitement des données

# Fonction appelée lorsqu'une touche directionnelle est pressée
def on_key(event):
    key = event.keysym
    print("Touche pressée:", key)
    # Code pour traiter les touches directionnelles
    # Remplacer cette partie par votre propre logique de traitement des touches

# Création de la fenêtre principale
root = tk.Tk()
root.title("Contrôle du Gyropode")

# Cadre pour les sliders
frame = ttk.Frame(root)
frame.pack(padx=10, pady=10)

# Labels et sliders pour les valeurs
nom_label = ttk.Label(frame, text="Nom:")
nom_label.grid(row=0, column=0, padx=5, pady=5)
nom_entry = ttk.Entry(frame)
nom_entry.grid(row=0, column=1, padx=5, pady=5)

val1_label = ttk.Label(frame, text="Val1:")
val1_label.grid(row=1, column=0, padx=5, pady=5)
val1_slider = ttk.Scale(frame, from_=0, to=100, orient="horizontal")
val1_slider.grid(row=1, column=1, padx=5, pady=5)

val2_label = ttk.Label(frame, text="Val2:")
val2_label.grid(row=2, column=0, padx=5, pady=5)
val2_slider = ttk.Scale(frame, from_=0, to=100, orient="horizontal")
val2_slider.grid(row=2, column=1, padx=5, pady=5)

val3_label = ttk.Label(frame, text="Val3:")
val3_label.grid(row=3, column=0, padx=5, pady=5)
val3_slider = ttk.Scale(frame, from_=0, to=100, orient="horizontal")
val3_slider.grid(row=3, column=1, padx=5, pady=5)

# Bouton d'envoi
send_button = ttk.Button(root, text="Envoyer", command=send_data)
send_button.pack(pady=5)

# Démarrer le thread pour la réception de données Bluetooth
receive_thread = threading.Thread(target=receive_data)
receive_thread.start()

# Associer la fonction on_key à l'événement des touches directionnelles
root.bind("<KeyPress>", on_key)

# Lancement de la boucle principale
root.mainloop()
