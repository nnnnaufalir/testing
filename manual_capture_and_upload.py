import time
import io
import os
from datetime import datetime # ### TAMBAHAN BARU ### (Untuk info Day, Date, Time)

# --- Import Library Kunci ---
import cv2
import numpy as np
from PIL import Image
import cloudinary
import cloudinary.uploader
import firebase_admin
from firebase_admin import credentials, db

# --- 1. KONFIGURASI (Tidak berubah) ---
cloudinary.config(
    cloud_name="deb3ckbz9",  ### GANTI INI ###
    api_key="886281278537257",        ### GANTI INI ###
    api_secret="f5Gj-1VdNHlPOSeR8OIUQ8qRwTI"     ### GANTI INI ###
)
cred = credentials.Certificate('firebase-key.json') 
firebase_admin.initialize_app(cred, {
    # (Ganti dengan URL Realtime Database Anda)
    'databaseURL': 'https://test-asv-monitoring-unnes-default-rtdb.asia-southeast1.firebasedatabase.app/'  ### GANTI INI ###
})
ref = db.reference('/kapal/tim-asv-01')

# --- 2. FUNGSI HELPER (Upload & Arsip - Tidak berubah) ---

def upload_opencv_frame_to_cloudinary(frame):
    # ... (Fungsi ini tidak berubah dari sebelumnya) ...
    try:
        print("\n[UPLOAD] Mengonversi frame OpenCV BGR ke RGB...")
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(frame_rgb)
        img.thumbnail((800, 800))
        buffer = io.BytesIO()
        img.save(buffer, format="JPEG", quality=80)
        buffer.seek(0)
        print("[UPLOAD] Meng-upload gambar terkompresi ke Cloudinary...")
        hasil_upload = cloudinary.uploader.upload(buffer, folder="asv_lomba")
        url = hasil_upload.get('secure_url')
        print(f"[UPLOAD] Upload sukses. URL: {url}")
        return url
    except Exception as e:
        print(f"[UPLOAD] GAGAL meng-upload frame OpenCV: {e}")
        return None

def archive_and_reset_data():
    # ... (Fungsi ini tidak berubah dari sebelumnya) ...
    print("[INIT] Mengecek data lomba sebelumnya...")
    try:
        old_data = ref.get()
        if old_data:
            print("[INIT] Data lama ditemukan. Mengarsipkan...")
            timestamp_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            archive_ref = db.reference(f'/arsip/lomba_{timestamp_str}')
            archive_ref.set(old_data)
            ref.delete()
            print(f"[INIT] Arsip disimpan ke /arsip/lomba_{timestamp_str} dan dashboard live di-reset.")
            time.sleep(2) 
        else:
            print("[INIT] Dashboard sudah bersih.")
    except Exception as e:
        print(f"[INIT] Gagal melakukan arsip/reset: {e}")

# --- 3. ### TAMBAHAN BARU ###: FUNGSI GEOTAGGING ---

def draw_geotag_on_image(frame, sensor_data):
    """
    Menggambar info geotag (berdasarkan format Anda) langsung ke frame OpenCV.
    """
    print("[GEOTAG] Menambahkan geotag ke frame...")
    
    # Ambil data sensor saat ini
    sog_knots = sensor_data.get("sog", 0.0)
    sog_kmh = sog_knots * 1.852
    cog = sensor_data.get("cog", 0.0)
    lat = sensor_data.get("lat", 0.0)
    lon = sensor_data.get("lon", 0.0)
    
    # Ambil waktu saat ini
    now = datetime.now()
    
    # Format Hari, Tanggal, Waktu
    day_str = f"Day: {now.strftime('%a')}" # %a -> Sun, Mon, Tue
    date_str = f"Date: {now.strftime('%d/%m/%Y')}"
    time_str = f"Time: {now.strftime('%H:%M:%S')}"
    
    # Format Koordinat (Format A)
    lat_dir = "N" if lat >= 0 else "S"
    lon_dir = "E" if lon >= 0 else "W"
    coord_str = f"Coord: [{lat_dir} {abs(lat):.5f} {lon_dir} {abs(lon):.5f}]" # 5 angka desimal
    
    # Format SOG dan COG
    sog_str = f"SOG: {sog_knots:.1f} knot ({sog_kmh:.1f} km/h)"
    cog_str = f"COG: {cog:.1f} deg"
    
    # Kumpulkan semua teks
    text_lines = [day_str, date_str, time_str, coord_str, sog_str, cog_str]
    
    # --- Parameter untuk menggambar teks ---
    y = 80           # Posisi Y awal
    x = 30           # Posisi X
    line_spacing = 30  # Jarak antar baris
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.7
    font_color = (255, 255, 255) # Teks Putih
    bg_color = (0, 0, 0)         # Bayangan Hitam
    thickness = 2
    
    # Gambar setiap baris teks ke frame
    for line in text_lines:
        # Gambar bayangan (teks hitam) terlebih dahulu, sedikit bergeser
        cv2.putText(frame, line, (x + 1, y + 1), font, font_scale, bg_color, thickness)
        # Gambar teks putih di atasnya
        cv2.putText(frame, line, (x, y), font, font_scale, font_color, thickness)
        y += line_spacing # Pindah ke baris berikutnya
        
    return frame

# --- 4. DATA DUMMY (Tidak berubah) ---
dummy_data_sensor = {
    "pos_x": 12.5, "pos_y": 2.5, "heading": 90.0, "cog": 90.0,
    "sog": 0.0, "lat": -3.56734, "lon": 104.67235, "battery": 99.0
}
data_payload = {
    # ... (Struktur payload tidak berubah) ...
    "position_log": {"preparation": "In Progress", "start": "Pending", "floating_ball": 0, "surface_imaging": "Pending", "underwater_imaging": "Pending", "finish": "Pending"},
    "attitude": {"sog": 0.0, "cog": 0.0, "heading": 0.0}, "local_position": {"x": 0.0, "y": 0.0},
    "gps_location": {"lat": 0.0, "lon": 0.0}, "current_mission": "Preparation",
    "mission_images": {"surface": None, "underwater": None}, "track_id": "A",
    "race_start_timestamp": None, "race_finish_timestamp": None,
    "indicators": {"battery": 100, "last_update": None}
}

# --- 5. PROGRAM UTAMA (LOOP LIVE) ---
if __name__ == "__main__":
    
    archive_and_reset_data()
    
    cap = cv2.VideoCapture(0) # ### GANTI INI ### (Mungkin perlu 1, 2, dst.)
    if not cap.isOpened():
        print("[Error] Tidak bisa membuka kamera.")
        exit()
        
    print("\n--- [ KAPAL SIAP - VERSI GEOTAG ] ---")
    print("Menampilkan live feed kamera.")
    print("Tekan 'c' untuk CAPTURE & UPLOAD gambar surface (dengan geotag).")
    print("Tekan 'v' untuk CAPTURE & UPLOAD gambar underwater (dengan geotag).")
    print("Tekan 'q' untuk KELUAR.")
    
    last_send_time = time.time()
    capture_trigger = None 

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("[Error] Gagal membaca frame.")
                break
            
            # Tampilkan live feed
            cv2.imshow('Live Camera - (c) surface | (v) underwater | (q) quit', frame)
            
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                print("\n[MAIN] Perintah KELUAR diterima.")
                break
            elif key == ord('c'):
                print("\n[MAIN] Trigger CAPTURE SURFACE diterima!")
                capture_trigger = "surface"
            elif key == ord('v'):
                print("\n[MAIN] Trigger CAPTURE UNDERWATER diterima!")
                capture_trigger = "underwater"

            # --- Gerbang 1 Detik ---
            current_time = time.time()
            if current_time - last_send_time >= 1.0:
                last_send_time = current_time
                
                # --- A. AMBIL DATA SENSOR ANDA DI SINI ---
                # ### GANTI INI ###
                # Ganti 'dummy_data_sensor' dengan data asli Anda
                # (Kita biarkan data dummy untuk tes)
                dummy_data_sensor["heading"] = (dummy_data_sensor["heading"] + 1.5) % 360
                dummy_data_sensor["pos_x"] += 0.1
                
                # --- B. ISI PAYLOAD ---
                # (Mengisi 'data_payload' dengan data dari 'dummy_data_sensor')
                data_payload["local_position"]["x"] = dummy_data_sensor["pos_x"]
                data_payload["local_position"]["y"] = dummy_data_sensor["pos_y"]
                data_payload["attitude"]["heading"] = dummy_data_sensor["heading"]
                data_payload["attitude"]["cog"] = dummy_data_sensor["cog"]
                data_payload["attitude"]["sog"] = dummy_data_sensor["sog"]
                data_payload["gps_location"]["lat"] = dummy_data_sensor["lat"]
                data_payload["gps_location"]["lon"] = dummy_data_sensor["lon"]
                data_payload["indicators"]["battery"] = dummy_data_sensor["battery"]

                # --- C. ### MODIFIKASI ###: CEK TRIGGER CAPTURE ---
                if capture_trigger:
                    log_key = f"{capture_trigger}_imaging" 
                    
                    print(f"[MAIN] Memproses trigger: {log_key}...")
                    data_payload["position_log"][log_key] = "In Progress"
                    ref.set(data_payload) 
                    
                    # 1. Ambil data sensor TERBARU (penting untuk geotag)
                    # ### GANTI INI ###
                    # Saat produksi, Anda mungkin ingin mengambil data sensor
                    # tepat pada milidetik 'frame' ini diambil.
                    # Untuk saat ini, kita gunakan data dummy terakhir.
                    current_sensor_data = dummy_data_sensor 
                    
                    # 2. Gambar Geotag ke frame
                    geotagged_frame = draw_geotag_on_image(frame.copy(), current_sensor_data)
                    
                    # 3. Upload frame yang SUDAH digeotag
                    url = upload_opencv_frame_to_cloudinary(geotagged_frame)
                    
                    if url:
                        data_payload["mission_images"][capture_trigger] = url
                        data_payload["position_log"][log_key] = "Done"
                    else:
                        data_payload["position_log"][log_key] = "Failed"
                    
                    capture_trigger = None # Reset trigger

                # --- D. KIRIM DATA KE FIREBASE ---
                data_payload["indicators"]["last_update"] = time.time()
                ref.set(data_payload)
                
                print(f"[KIRIM] Data dikirim. Pos X: {data_payload['local_position']['x']:.1f}", end='\r')

    finally:
        # Bersihkan setelah selesai
        cap.release()
        cv2.destroyAllWindows()
        print("\n--- Skrip Kapal Selesai ---")