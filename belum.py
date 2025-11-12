import cv2
from ultralytics import YOLO
import time
import math
from pymavlink import mavutil
import threading # Import threading

# --- Import Pustaka Kode 2 ---
import io
import os
from datetime import datetime
import numpy as np
from PIL import Image
import cloudinary
import cloudinary.uploader
import firebase_admin
from firebase_admin import credentials, db

# ==============================================================================
# === 1. INISIALISASI (Gabungan dari Kode 1 & 2) ===
# ==============================================================================

# --- KONEKSI MAVLINK (Kode 1) ---
print("Menghubungkan ke vehicle...")
connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
connection.wait_heartbeat()
print("Connected to vehicle!")

# --- KONEKSI CLOUD (Kode 2) ---
cloudinary.config(cloud_name="...", api_key="...", api_secret="...")
cred = credentials.Certificate('firebase-key.json')
firebase_admin.initialize_app(cred, {'databaseURL': '...'})
ref = db.reference('/kapal/tim-asv-01')

# --- LOAD MODEL (Kode 1) ---
MODEL_PATH = '../models/best.egine'
model = YOLO(MODEL_PATH)

# --- KONSTANTA (Gabungan) ---
FRAME_WIDTH, FRAME_HEIGHT = 640, 640
# ... (masukkan class_names, pivot_point, rc_values Anda di sini) ...
rc3_value_lurus = 1500
rc1_value_awal = 1500 
rc2_value_awal = 1500
rc4_value_awal = 1500
# ... (dan seterusnya) ...


# ==============================================================================
# === 2. SEMUA FUNGSI HELPER ANDA DI SINI ===
# ==============================================================================

# --- Fungsi dari Kode 1 ---
def override_rc_channels(connection, rc1, rc2, rc3, rc4):
    # ... (logika override_rc_channels Anda) ...
    pass

def get_vehicle_data():
    # ... (logika lengkap get_vehicle_data Anda) ...
    # Pastikan untuk menangani timeout jika pesan tidak kunjung datang
    # agar tidak memblokir loop utama. Ganti blocking=True menjadi
    # blocking=False dengan timeout, atau atur timeout di connection.
    # Contoh sederhana (idealnya gunakan timeout):
    try:
        msg_gps = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=0.1)
        # ... (lakukan ini untuk semua msg) ...
        # if not msg_gps: return None # Kembalikan None jika data tidak lengkap
        # ... (sisa logika Anda) ...
        return {} # Kembalikan data dictionary
    except Exception as e:
        print(f"Gagal get_vehicle_data: {e}")
        return None # Kembalikan None jika gagal

def calculate_mid_point(point1, point2):
    # ... (logika Anda) ...
    return ((point1[0] + point2[0]) / 2, (point1[1] + point2[1]) / 2)

# --- Fungsi dari Kode 2 ---
def upload_opencv_frame_to_cloudinary(frame):
    # ... (logika upload Anda) ...
    pass

def draw_geotag_on_image(frame, sensor_data):
    # ... (logika geotag Anda) ...
    pass

def archive_and_reset_data():
    # ... (logika arsip Anda) ...
    pass


# ==============================================================================
# === 3. LOGIKA THREADING (INTI BARU) ===
# ==============================================================================

# Variabel global untuk berbagi data antar thread
# Kita gunakan dictionary agar mudah di-lock
shared_data = {
    "telemetry": None,
    "monitoring_frame": None,
    "capture_trigger": None
}
# Lock untuk memastikan tidak ada data korup saat diakses bersamaan
data_lock = threading.Lock()

# Ini adalah LOGIKA KODE 2, dibungkus sebagai fungsi worker
def monitoring_thread_task():
    """
    Fungsi ini berjalan di thread terpisah.
    Tugasnya hanya mengirim data setiap 1 detik.
    """
    print("[Worker Thread] Siap. Menunggu data telemetri...")
    
    # Siapkan data payload sekali saja
    data_payload = {
        "position_log": {"preparation": "In Progress", "start": "Pending", "floating_ball": 0, "surface_imaging": "Pending", "underwater_imaging": "Pending", "finish": "Pending"},
        "attitude": {"sog": 0.0, "cog": 0.0, "heading": 0.0}, "local_position": {"x": 0.0, "y": 0.0},
        "gps_location": {"lat": 0.0, "lon": 0.0}, "current_mission": "Preparation",
        "mission_images": {"surface": None, "underwater": None}, "track_id": "A",
        "race_start_timestamp": None, "race_finish_timestamp": None,
        "indicators": {"battery": 100, "last_update": None}
    }

    while True:
        # Ambil salinan data dengan aman
        with data_lock:
            current_telemetry = shared_data["telemetry"]
            current_frame = shared_data["monitoring_frame"]
            trigger = shared_data["capture_trigger"]
            shared_data["capture_trigger"] = None # Langsung reset trigger

        # --- A. PROSES PENGIRIMAN TELEMETRI (SETIAP 1 DETIK) ---
        if current_telemetry:
            try:
                # Isi payload dari data telemetri terbaru
                data_payload["gps_location"]["lat"] = current_telemetry["gps"]['lat']
                data_payload["gps_location"]["lon"] = current_telemetry["gps"]['lon']
                data_payload["attitude"]["sog"] = current_telemetry["speed"]['ground_speed']
                data_payload["attitude"]["cog"] = current_telemetry["gps"]['cog']
                # ... (isi sisa payload Anda) ...
                data_payload["indicators"]["battery"] = current_telemetry["bat_status"]['level']
                
                # --- B. PROSES CAPTURE JIKA ADA TRIGGER ---
                if trigger and current_frame is not None:
                    print(f"[Worker Thread] Memproses trigger: {trigger}...")
                    
                    # 1. Geotag (menggunakan data telemetri yang sama)
                    geotagged_frame = draw_geotag_on_image(current_frame.copy(), current_telemetry["gps"]) # Sesuaikan data yg dikirim
                    
                    # 2. Upload (Operasi Lambat)
                    url = upload_opencv_frame_to_cloudinary(geotagged_frame)
                    
                    if url:
                        data_payload["mission_images"][trigger] = url
                        data_payload["position_log"][f"{trigger}_imaging"] = "Done"
                    else:
                        data_payload["position_log"][f"{trigger}_imaging"] = "Failed"

                # --- C. KIRIM KE FIREBASE (Operasi Lambat) ---
                data_payload["indicators"]["last_update"] = time.time()
                ref.set(data_payload)
                # print("[Worker Thread] Data telemetri terkirim.") # Uncomment untuk debug

            except Exception as e:
                print(f"[Worker Thread] Error: {e}")
        
        # Tunggu 1 detik sebelum iterasi berikutnya
        time.sleep(1.0)


# ==============================================================================
# === 4. FUNGSI MAIN (LOOP OTONOM) ===
# ==============================================================================

def main_loop():
    """
    Ini adalah LOGIKA KODE 1.
    Berjalan di Main Thread secepat mungkin (FPS tinggi).
    """
    
    # Buka DUA kamera
    # Ganti 0 dan 1 sesuai dengan port kamera Anda
    cap_otonom = cv2.VideoCapture(VIDEO_PATH) # Kamera untuk YOLO (bisa juga file)
    cap_monitor = cv2.VideoCapture(1)         # Kamera untuk Geotag/Monitoring
    
    if not cap_otonom.isOpened():
        print("Error: Tidak bisa buka kamera Otonom.")
        return
    if not cap_monitor.isOpened():
        print("Error: Tidak bisa buka kamera Monitor.")
        return

    print("[Main Thread] Loop otonom dimulai...")
    
    mid_point = (320, 320) # Nilai default

    try:
        while True:
            # --- 1. BACA INPUT (Cepat) ---
            ret_o, frame_otonom = cap_otonom.read()
            ret_m, frame_monitor = cap_monitor.read()
            
            if not ret_o or not ret_m:
                print("[Main Thread] Gagal baca frame, loop berhenti.")
                break
                
            # Resize frame otonom untuk YOLO
            frame_otonom_resized = cv2.resize(frame_otonom, (FRAME_WIDTH, FRAME_HEIGHT))

            # --- 2. DAPATKAN DATA TELEMETRI (Cepat) ---
            # Panggil fungsi ini di setiap loop!
            vehicle_data = get_vehicle_data() 

            # --- 3. LOGIKA OTONOM (Kode 1) (Cepat) ---
            start_time = time.time()
            results = model.predict(source=frame_otonom_resized, imgsz=640, conf=0.6, device='0', verbose=False)
            fps = 1 / (time.time() - start_time)

            # ... (Logika Anda untuk mencari closest_green_ball, closest_red_ball) ...
            closest_green_ball = None
            closest_red_ball = None
            
            # (Loop deteksi Anda)
            for result in results:
                for box in result.boxes:
                    # ... (logika ekstraksi box Anda) ...
                    pass
            
            if closest_green_ball and closest_red_ball:
                 # ... (logika hitung mid_point Anda) ...
                 pass

            # ... (Logika visualisasi Anda ke frame_otonom_resized) ...
            # draw_center_line(frame_otonom_resized, fps, ..., mid_point)

            # --- 4. LOGIKA KENDALI (Kode 1) (Cepat) ---
            if mid_point[0] > 250 and mid_point[0] < 390:
                override_rc_channels(connection, rc1_value_awal, rc2_value_awal, rc3_value_lurus, rc4_value_awal)
            # ... (elif lainnya) ...
            
            # --- 5. UPDATE DATA UNTUK WORKER THREAD (Sangat Cepat) ---
            with data_lock:
                if vehicle_data:
                    shared_data["telemetry"] = vehicle_data
                shared_data["monitoring_frame"] = frame_monitor

            # --- 6. TAMPILKAN VISUAL & CEK KEYBOARD (Cepat) ---
            cv2.imshow('Otonom (Kamera 0)', frame_otonom_resized)
            cv2.imshow('Monitoring (Kamera 1)', frame_monitor)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("[Main Thread] Perintah KELUAR diterima.")
                break
            
            # Set trigger untuk worker thread
            elif key == ord('c'):
                print("[Main Thread] Trigger CAPTURE SURFACE diterima!")
                with data_lock:
                    shared_data["capture_trigger"] = "surface"
            elif key == ord('v'):
                print("[Main Thread] Trigger CAPTURE UNDERWATER diterima!")
                with data_lock:
                    shared_data["capture_trigger"] = "underwater"

    finally:
        # Cleanup
        print("[Main Thread] Membersihkan...")
        cap_otonom.release()
        cap_monitor.release()
        cv2.destroyAllWindows()
        connection.close()
        print("[Main Thread] Selesai.")


# ==============================================================================
# === 5. EKSEKUSI PROGRAM UTAMA ===
# ==============================================================================

if __name__ == "__main__":
    
    # 1. Bersihkan data lama di Firebase
    archive_and_reset_data()
    
    # 2. Buat dan jalankan Worker Thread (Monitoring)
    # daemon=True berarti thread ini akan otomatis mati jika program utama selesai
    monitor_thread = threading.Thread(target=monitoring_thread_task, daemon=True)
    monitor_thread.start()
    
    # 3. Jalankan Main Loop (Otonom) di thread utama
    main_loop()
