# DRONE-ROS2-CONTROLLER


Dokumentacja techniczna i instrukcja wdrożeniowa projektu autonomicznego drona kołowego opartego na platformie Raspberry Pi 4 oraz systemie ROS 2 Jazzy Jalisco. Układ sterowania wykorzystuje algorytmy SLAM do mapowania przestrzeni oraz holonomiczny napęd oparty na kołach typu Mecanum. Manipulacja wektorami ruchu realizowana jest za pomocą interfejsu przeglądarkowego lub bezprzewodowego kontrolera PlayStation 5 DualSense.

---

## Wymagania sprzętowe
* **Komputer pokładowy:** Raspberry Pi 4 Model B.
* **System operacyjny:** Ubuntu Server 24.04 LTS (64-bit).
* **Głowica skanująca:** LiDAR 360 stopni podłączony poprzez konwerter szeregowy USB (domyślnie `/dev/ttyUSB0`).
* **Kontroler:** Pad PlayStation 5 (DualSense) z modułem Bluetooth.
* **Układ wykonawczy:** Podwozie 4x4 Mecanum wraz z dedykowanym sterownikiem silników.

---

## 1. Przygotowanie systemu operacyjnego i pierwsze uruchomienie

### Krok 1.1: Flashowanie karty pamięci SD
1. Uruchom narzędzie Raspberry Pi Imager na komputerze stacjonarnym.
2. Wybierz urządzenie docelowe: Raspberry Pi 4.
3. Wybierz system operacyjny: Other general-purpose OS -> Ubuntu -> Ubuntu Server 24.04 LTS (64-bit).
4. Przejdź do edycji ustawień zaawansowanych (ikona koła zębatego):
   * W zakładce ogólnej zdefiniuj nazwę użytkownika jako `ubuntu` i przypisz hasło dostępowe.
   * W zakładce usług zaznacz pole "Włącz SSH" i wybierz autoryzację hasłem.
5. Zatwierdź operację i dokonaj zapisu obrazu na karcie SD.

### Krok 1.2: Identyfikacja adresu sieciowego IP
Włóż przygotowaną kartę pamięci do slotu Raspberry Pi 4, podłącz zasilanie układu i odczekaj 120 sekund na inicjalizację usług sieciowych. W terminalu komputera klienckiego (podłączonego do tej samej podsieci) wykonaj komendę:

```bash
ping -4 ubuntu.local
```
Zanotuj zwrócony przez system adres IP (np. `192.168.1.15`).

### Krok 1.3: Konfiguracja połączenia z Visual Studio Code
1. Uruchom edytor VS Code na komputerze klienckim i zainstaluj oficjalne rozszerzenie: Remote - SSH.
2. Wywołaj pasek narzędzi skrótem `F1`, wpisz frazę `Remote-SSH: Connect to Host...` i zatwierdź klawiszem Enter.
3. Wprowadź ciąg połączenia: `ssh ubuntu@<ZNALEZIONY_ADRES_IP>`.
4. Wybierz środowisko docelowe jako Linux i podaj hasło użytkownika `ubuntu`.
5. Uruchom terminal wewnątrz edytora (Terminal -> New Terminal).
6. W panelu bocznym wybierz opcję Open Folder i wskaż ścieżkę `/home/ubuntu/`.

---

## 2. Trwała konfiguracja interfejsów sieciowych oraz Bluetooth

### Krok 2.1: Implementacja menedżera sieci NetworkManager
Aby system automatycznie nawiązywał połączenie z siecią Wi-Fi przy każdym rozruchu, należy zmodyfikować domyślny podsystem sieciowy:

```bash
sudo apt update
sudo apt install network-manager -y
sudo nano /etc/netplan/50-cloud-init.yaml
```
Wewnątrz pliku konfiguracyjnego, bezpośrednio w bloku sekcji `network:`, dopisz definicję renderera z zachowaniem formatowania yaml (dwie spacje wcięcia):

```yaml
renderer: NetworkManager
```
Zapisz zmiany (`Ctrl+O`, Enter) i zamknij plik (`Ctrl+X`). Zastosuj nowe reguły poleceniem:

```bash
sudo netplan apply
```
> **Uwaga:** Zmiana podsystemu może chwilowo zerwać aktywne połączenie SSH. W takim przypadku odczekaj chwilę i połącz się ponownie.

W celu dodania nowej sieci Wi-Fi w trybie headless, wykonaj komendę bezpośrednią:

```bash
sudo nmcli device wifi connect "NAZWA_SIECI" password "HASLO_DO_SIECI"
```
Wymuś automatyczne łączenie z wybranym punktem dostępowym przy każdym restarcie:

```bash
sudo nmcli connection modify "NAZWA_SIECI" connection.autoconnect yes
```

### Krok 2.2: Parowanie i nadawanie flagi zaufania dla kontrolera PS5
Konfiguracja automatycznego łączenia pada poprzez wbudowany stos Bluetooth:

```bash
bluetoothctl
```
Po przejściu do powłoki konfiguracyjnej `[bluetooth]#` wpisz sekwencję komend przygotowawczych:

```bash
agent on
default-agent
scan on
```
Wprowadź pad DualSense w tryb parowania: przytrzymaj jednocześnie przycisk **Create** oraz przycisk **PS** do momentu szybkiego, pulsacyjnego migania diod wokół panelu dotykowego.

W oknie terminala pojawi się adres MAC wykrytego urządzenia o nazwie Wireless Controller (format `XX:XX:XX:XX:XX:XX`). Skopiuj go i wykonaj procedurę autoryzacji:

```bash
scan off
pair ADRES_MAC
trust ADRES_MAC
connect ADRES_MAC
quit
```
Prawidłowe zakończenie procesu sygnalizowane jest stałym podświetleniem diody na kontrolerze. Urządzenie będzie odtąd automatycznie łączyć się z Raspberry Pi po naciśnięciu przycisku PS.

---

## 3. Instalacja platformy ROS 2 Jazzy oraz zależności systemowych

### Krok 3.1: Dodanie repozytoriów i instalacja pakietu bazowego
Wykonaj poniższy zestaw instrukcji w celu wdrożenia oficjalnych kluczy oraz pakietu `ros-base`:

```bash
sudo apt update && sudo apt install software-properties-common curl -y
sudo add-apt-repository universe -y
sudo curl -sSL [https://raw.githubusercontent.com/ros/rosdistro/master/ros.key](https://raw.githubusercontent.com/ros/rosdistro/master/ros.key) -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] [http://packages.ros.org/ros2/ubuntu](http://packages.ros.org/ros2/ubuntu) $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-jazzy-ros-base -y
```

### Krok 3.2: Instalacja binarnych bibliotek robotycznych oraz modułów Pythona
Pobierz pakiety odpowiedzialne za przetwarzanie danych z sensora, estymację odometrii i integrację ze stosem nawigacyjnym:

```bash
sudo apt install -y ros-jazzy-slam-toolbox ros-jazzy-sllidar-ros2 ros-jazzy-rf2o-laser-odometry ros-jazzy-foxglove-bridge ros-jazzy-robot-state-publisher python3-colcon-common-extensions git
```
Wdróż pakiety Pythona odpowiedzialne za hosting serwera Web oraz przechwytywanie zdarzeń systemowych z poziomu kontrolera DualSense:

```bash
pip install flask evdev --break-system-packages
```

---

## 4. Strukturyzacja katalogów projektu i dystrybucja plików

### Krok 4.1: Klonowanie repozytorium kodu źródłowego
Utwórz strukturę folderów roboczych i pobierz komponenty oprogramowania:

```bash
mkdir -p ~/lidar_slam_ws/maps
cd ~
git clone [https://github.com/radekplichta44/DRON-ROS2-CONTROLLER.git](https://github.com/radekplichta44/DRON-ROS2-CONTROLLER.git)
```

### Krok 4.2: Przetransferowanie konfiguracji i skryptów automatyzujących
Zgodnie z wymaganiami lokalizacyjnymi poszczególnych pakietów ROS 2, rozdziel pobrane pliki za pomocą masek rozszerzeń do dedykowanego obszaru roboczego `lidar_slam_ws`:

```bash
mv ~/DRON-ROS2-CONTROLLER/*.yaml ~/lidar_slam_ws/
mv ~/DRON-ROS2-CONTROLLER/moj_robot.urdf ~/lidar_slam_ws/
mv ~/DRON-ROS2-CONTROLLER/start_mapping.sh ~/lidar_slam_ws/
mv ~/DRON-ROS2-CONTROLLER/start_mapping_localization.sh ~/lidar_slam_ws/
mv ~/DRON-ROS2-CONTROLLER/zapisz_mape.sh ~/lidar_slam_ws/
```
W katalogu źródłowym `~/DRON-ROS2-CONTROLLER` pozostają wyłącznie pliki rdzenia aplikacyjnego: `pilot.py` oraz `start_dron.sh`.

### Krok 4.3: Zabezpieczenie uprawnień uruchomieniowych i sprzętowych
Przypisz atrybuty wykonywalności dla skryptów powłoki systemowej i nadaj uprawnienia magistrali szeregowej USB dla LiDARu:

```bash
chmod +x ~/lidar_slam_ws/*.sh
chmod +x ~/DRON-ROS2-CONTROLLER/*.sh
sudo chmod 777 /dev/ttyUSB0
```

---

## 5. Procedura eksploatacyjna

### Krok 5.1: Rozruch aplikacji nadrzędnej
Przejdź do lokalizacji repozytorium sterownika i wywołaj skrypt główny:

```bash
cd ~/DRON-ROS2-CONTROLLER
python3 pilot.py
```

### Krok 5.2: Obsługa interfejsu przeglądarkowego WWW
Wpisz w pasku adresowym urządzenia znajdującego się w tej samej podsieci adres serwera Flask:

```text
http://<ADRES_IP_RASPBERRY_PI>:5000
```
Poziom UI udostępnia funkcjonalności mapowania terenu, kontroli wektora poruszania się oraz manipulacji progami prędkości bazowych.

### Krok 5.3: Specyfikacja mapowania przycisków kontrolera (Master Override)
Inicjalizacja transmisji z pada DualSense natychmiastowo odłącza żądania wysyłane z poziomu interfejsu HTML.

* **Gałki analogowe:** Płynna translacja wektora prędkości (jazda holonomiczna przód/tył/boki oraz obrót w osi pionowej robot-frame).
* **Krzyżak (D-Pad):** Modyfikacja dyskretna mnożników prędkości.
* **Kółko:** Wywołanie skryptu `start_mapping.sh` (tworzenie mapy od podstaw).
* **Krzyżyk:** Natychmiastowe przerwanie procesów SLAM i wstrzymanie emisji wiązki lasera.
* **Trójkąt:** Wywołanie procedury `zapisz_mape.sh` (zrzut danych do formatu `.pgm` oraz `.yaml`).

### Krok 5.4: Konfiguracja mostu telemetrycznego Foxglove Studio
1. W dolnej sekcji środowiska edytorskiego VS Code przejdź do modułu **Ports**.
2. Wybierz opcję **Add Port** i wprowadź wartość portu sieciowego systemu ROS: **8765**.
3. Uruchom zewnętrzną aplikację Foxglove Studio na maszynie klienckiej.
4. Wybierz typ połączenia: **Foxglove WebSocket** i podaj adres nasłuchu:

```text
ws://<ADRES_IP_RASPBERRY_PI>:8765
```
