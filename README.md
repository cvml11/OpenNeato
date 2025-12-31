# OpenNeato

![License](https://img.shields.io/badge/License-GPLv3-blue.svg)
![ROS 2 Version](https://img.shields.io/badge/ROS_2-Jazzy_Jalisco-green.svg)
![Platform](https://img.shields.io/badge/Platform-Radxa_Zero_3W-orange.svg)
![Status](https://img.shields.io/badge/Status-Alpha-red.svg)

**OpenNeato** è un progetto open source dedicato a rivitalizzare il robot aspirapolvere **Neato D7 Botvac** sostituendo la logica proprietaria obsoleta con hardware moderno e software libero.

L'obiettivo è trasformare il Neato D7 in una piattaforma robotica di ricerca e domotica completamente controllabile, sfruttando la potenza di **ROS 2 Jazzy** su un **Radxa Zero 3W**.

---

## ⚠️ Avvertenze di Sicurezza (Safety Disclaimer)

**LEGGERE ATTENTAMENTE PRIMA DI PROCEDERE**

1.  **Rischio Elettrico e Incendio:** Questo progetto richiede la modifica dell'hardware interno del robot. Il Neato D7 utilizza batterie agli ioni di litio (Li-Ion) ad alta capacità. Un cablaggio errato, cortocircuiti o una gestione impropria della batteria possono causare incendi, esplosioni o lesioni gravi.
2.  **Invalidamento Garanzia:** L'apertura del dispositivo e qualsiasi modifica hardware o firmware invalidano immediatamente qualsiasi garanzia residua del produttore.
3.  **Nessuna Responsabilità:** Il software e le istruzioni sono forniti "COSÌ COME SONO", senza alcuna garanzia. Gli autori e i contributori di OpenNeato non sono responsabili per danni al dispositivo, alla proprietà o alle persone derivanti dall'uso di questo progetto. Procedi a tuo rischio e pericolo.

---

## 🛒 Bill of Materials (Hardware Richiesto)

Per completare la conversione sono necessari i seguenti componenti:

*   **Computer di Bordo:** Radxa Zero 3W (o equivalente ARM64 compatibile con Ubuntu 24.04).
*   **Robot:** Neato D7 Botvac (funzionante meccanicamente).
*   **Alimentazione:** Convertitore DC-DC Buck (per abbassare la tensione della batteria del Neato a 5V per il Radxa).
*   **Connettività:** Cavi UART/USB per interfacciare il Radxa alla porta seriale della scheda madre del Neato (o direttamente al LiDAR/motori se si bypassa la PCB originale).
*   **MicroSD:** Classe 10, min 32GB per l'OS.

---

## 🚀 Installation

L'installazione è automatizzata tramite script per configurare l'ambiente su Ubuntu 24.04.

1.  Clona il repository sul tuo Radxa Zero 3W:
    ```bash
    git clone https://github.com/tuo-username/OpenNeato.git
    cd OpenNeato
    ```

2.  Esegui lo script di installazione:
    ```bash
    cd installer
    sudo ./install.sh
    ```

Lo script si occuperà di:
*   Installare le dipendenze di sistema.
*   Configurare l'ambiente ROS 2 Jazzy.
*   Installare le dipendenze Python per la Web Interface.
*   Configurare i servizi systemd per l'avvio automatico.

---

## 📄 License

Questo progetto è rilasciato sotto la licenza **GNU General Public License v3.0 (GPLv3)**.

Ciò significa che:
*   ✅ Puoi usare, copiare e modificare il software liberamente.
*   ✅ Se distribuisci versioni modificate, **DEVI** rilasciare il codice sorgente sotto la stessa licenza (GPLv3).
*   🚫 Non puoi chiudere il codice sorgente o utilizzarlo in prodotti commerciali proprietari senza rilasciare le modifiche.

Vedi il file [LICENSE](LICENSE) per il testo completo.
