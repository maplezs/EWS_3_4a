# Manual Penggunaan Alat Robot Arm 3-DOF Kendali LQR
# EWS_3_4A
## 1. Penggunaan Alat
- Hubungkan kabel USB ke mikrokontroler pada wadah sirkuit
- Hubungkan konektor gripper dari sirkuit ke alat lengan robot
- Hubungkan konektor Servo Dynamixel dengan kabel pada sirkuit
- Hubungkan kabel USB ke komputer


## 2. Penggunaan Aplikasi Kalkulasi LQR
### Sebelum menggunakan aplikasi, pastikan MATLAB versi R2020 atau lebih baru, sudah terinstal
- Buka langsung file aplikasi yang berformat .mlapp
- Aplikasi MATLAB akan terbuka, tunggu hingga jendela aplikasi Kalkulasi LQR muncul
- Atur Nilai Parameter tabel matriks Q dan R sesuai keinginan
- Klik tombol kalkulasi untuk melakukan kalkulasi nilai gain
- Setelah kalkulasi selesai, tabel matriks K (gain) akan terisi otomatis
- Untuk menyalin seluruh nilai dari suatu baris, klik salah satu nilai pada baris yang akan disalin, lalu tekan Ctrl + C (Windows) atau Command-C (MacOS)

## 3. Penggunaan Aplikasi Robot Arm Control 
### Sebelum menggunakan aplikasi, pastikan Python versi 3.11 sudah terinstal
- Unduh aplikasi melalui [Link ini](https://github.com/maplezs/EWS_3_4a/archive/refs/heads/main.zip)
- Ekstrak arsip tersebut
- Buka terminal / command prompt / powershell
- Jalankan perintah berikut
```
pip install -r requirements.txt
```
- Setelah semua pustaka yang dibutuhkan berhasil diinstal 
- Klik dua kali pada file **new_window_invers.py**
- Atau pada terminal / command prompt / powershell jalankan perintah berikut
```
python new_window_invers.py
```
- Jendela aplikasi akan muncul
- Pilih port, jika mikrokontroler belum dihubungkan saat aplikasi berjalan, hubungkan mikrokontroler lalu tekan tombol "Refresh" untuk memindai ulang port yang terdeteksi pada komputer
- Setelah memilih port, klik tombol "hubungkan" untuk mencoba terhubung dengan mikrokontroler
- Jendela dialog berhasil terhubung akan muncul jika sukses terhubung, klik OK
- Jendela akan menjadi lebih luas dan memunculkan semua menu aplikasi
### Manual Aplikasi Robot Arm Control
- Pilih port, jika mikrokontroler belum dihubungkan saat aplikasi berjalan, hubungkan mikrokontroler lalu tekan tombol "Refresh" untuk memindai ulang port yang terdeteksi pada komputer
- Setelah memilih port, klik tombol "hubungkan" untuk mencoba terhubung dengan mikrokontroler
- Jendela dialog berhasil terhubung akan muncul jika sukses terhubung, klik OK
- Jendela akan menjadi lebih luas dan memunculkan semua menu aplikasi
#### Urutan cara penggunaan aplikasi
1. Membuka data konfigurasi yang tersimpan (opsional)
2. Tentukan jumlah iterasi
3. Tentukan koordinat *waypoint trajectory* sesuai jumlah iterasi
4. Mengisi dengan menempel nilai matriks K (gain) yang diperoleh dari hasil kalkulasi aplikasi Kalkulasi LQR
5. Menentukan iterasi untuk membuka tutup gripper atau tidak menggunakan gripper 
6. Menyimpan data konfigurasi (opsional)
7. Klik tombol "kirim" untuk mengirimkan data ke mikrokontroler
8. Mikrokontroler akan mengirimkan data persentase nilai torsi dan *trajectory* dari tiap servo Dynamixel sesuai dengan jumlah iterasi
9. Setelah robot menyelesaikan seluruh iterasi, tombol "plot" dapat digunakan untuk menampilkan plot grafik torsi dari tiap servo dan perbadingan *trajectory* referensi dengan aktual