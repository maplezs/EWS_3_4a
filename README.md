# Manual Penggunaan Alat Robot Arm 3-DOF Kendali LQR
# EWS_3_4A
## 1. Penggunaan Alat
- Hubungkan kabel USB ke mikrokontroler
- Hubungkan konektor gripper dari sirkuit ke alat lengan robot
- Hubungkan konektor Servo Dynamixel dengan kabel pada sirkuit
- Hubungkan 2 power supply 12V ke alat lengan robot
- Hubungkan kabel USB ke komputer

## 2. Penggunaan Aplikasi Kalkulasi LQR
### Sebelum menggunakan aplikasi, pastikan MATLAB versi R2022a atau lebih baru sudah terinstal
- Buka file aplikasi yang berformat **.mlapp**
- Aplikasi MATLAB akan terbuka, tunggu hingga jendela aplikasi Kalkulasi LQR muncul
- Atur Nilai Parameter tabel matriks Q dan R sesuai keinginan
- Klik tombol **Kalkulasi** untuk melakukan kalkulasi nilai gain
- Setelah kalkulasi selesai, tabel matriks K (gain) akan terisi otomatis
- Untuk menyalin seluruh nilai dari satu baris, klik salah satu nilai pada baris yang akan disalin, lalu tekan **Ctrl + C** (Windows) atau **Command-C** (MacOS)

## 3. Penggunaan Aplikasi Robot Arm Control 
### Jika menggunakan Windows, ikuti langkah dibawah ini
- Unduh aplikasi melalui [Link ini](https://drive.google.com/file/d/17Mk4Lwvuc8YeCvbq_F39dM77tmaTnbs5/view?usp=sharing)
- Ekstrak arsip tersebut
- Buka dan jalankan aplikasi **RobotArmControlApp.exe**
### Untuk MacOS dan Linux, sebelum menggunakan aplikasi, pastikan Python versi 3.11 sudah terinstal
- Unduh aplikasi melalui [Link ini](https://github.com/maplezs/EWS_3_4a/archive/refs/heads/main.zip)
- Ekstrak arsip tersebut
- Buka terminal / command prompt / powershell
- Jalankan perintah berikut
```
pip install -r requirements.txt
```
- Pastikan semua library berhasil terinstal
- Klik dua kali pada file **RobotArmControlApp.py** atau pada terminal / command prompt / powershell jalankan perintah berikut
```
python RobotArmControlApp.py
```
- Jendela aplikasi akan muncul
### Manual Aplikasi Robot Arm Control
- Pilih port, jika mikrokontroler belum dihubungkan saat aplikasi berjalan, hubungkan mikrokontroler lalu tekan tombol **Refresh** untuk memindai ulang port yang terdeteksi pada komputer
- Setelah memilih port, klik tombol **Connect** untuk mencoba terhubung dengan mikrokontroler
- Jendela dialog berhasil terhubung akan muncul jika sukses terhubung, klik OK
- Jendela akan menjadi lebih luas dan memunculkan semua menu aplikasi
#### Urutan cara penggunaan aplikasi
- Membuka data konfigurasi yang tersimpan (opsional) tekan kombinasi tombol **Ctrl + O** atau pada menu bar **File** pilih **Open config**
- Tentukan jumlah iterasi
- Tentukan koordinat waypoint trajectory sesuai jumlah iterasi
- Mengisi dengan menempel dengan **Ctrl+V** (Windows) atau **Command-V** (MacOS) nilai matriks K (gain) yang diperoleh dari hasil kalkulasi aplikasi Kalkulasi LQR
- Menentukan iterasi untuk membuka tutup gripper atau tidak menggunakan gripper 
- Menyimpan data konfigurasi (opsional) tekan kombinasi tombol **Ctrl + S** atau pada menu bar **File** pilih **Save config**
- Klik tombol **Send** untuk mengirimkan data ke mikrokontroler
- Mikrokontroler akan mengirimkan data output torsi dan trajectory dari tiap aktuator joint sesuai dengan jumlah iterasi
- Jendela **System Response Plot** yang akan menampilkan plot real-time dari respon sistem
- Setelah robot menyelesaikan seluruh iterasi, tombol **Plot** dapat digunakan untuk menampilkan plot grafik torsi dari tiap servo atau perbadingan trajectory referensi dengan aktual
- Untuk menyimpan gambar plot respon sistem, klik kanan pada grafik lalu pilih export
- Untuk menyimpan log trajectory dan torsi tekan kombinasi tombol **Ctrl + Shift + S** atau pada menu bar **File** pilih **Save log**
## Screenshots

![App Screenshot 1](https://raw.githubusercontent.com/maplezs/EWS_3_4a/main/assets/screenshot1.png)

![App Screenshot 2](https://raw.githubusercontent.com/maplezs/EWS_3_4a/main/assets/screenshot2.png)

![App Screenshot 3](https://raw.githubusercontent.com/maplezs/EWS_3_4a/main/assets/screenshot3.png)


## Authors

- [@maplezs](https://www.github.com/maplezs)

