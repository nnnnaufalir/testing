# ==============================================================================
# === 1. IMPORT PUSTAKA (GABUNGAN) ===
# ==============================================================================
import cv2
from ultralytics import YOLO
import time
import math
from pymavlink import mavutil
import threading  # Untuk multithreading
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
# === 2. KONFIGURASI GLOBAL (GABUNGAN) ===
# ==============================================================================

# --- KONEKSI MAVLINK (Kode 1) ---
# [PERHATIAN] Ganti port dan baudrate jika perlu
print("Menghubungkan ke vehicle...")
try:
    connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
    connection.wait_heartbeat()
    print("Connected to vehicle!")
except Exception as e:
    print(f"Gagal terhubung ke MAVLink: {e}")
    print("Memulai program tanpa koneksi MAVLink (mode simulasi)...")
    connection = None  # Tetap jalan tanpa MAVLink untuk tes CV

# --- KONEKSI CLOUD (Kode 2) ---
# [PERHATIAN] Ganti dengan kredensial Anda
try:
    cloudinary.config(
        cloud_name="DEB3CKBZ9",
        api_key="886281278537257",
        api_secret="F5GJ-1VDNHLPOSE..."
    )
    cred = credentials.Certificate('firebase-key.json')
    firebase_admin.initialize_app(cred, {
        'databaseURL': 'https://test-asv-monitoring-unnes-default-rtdb.asia-southeast1.firebasedatabase.app/'
    })
    ref = db.reference('/kapal/tim-asv-01')
    print("Terhubung ke Firebase & Cloudinary.")
except Exception as e:
    print(f"Gagal terhubung ke Firebase/Cloudinary: {e}")
    ref = None

# --- KONSTANTA (Kode 1) ---
MODEL_PATH = '../models/best.egine'
# [PERHATIAN] Ganti VIDEO_PATH ke 0 untuk kamera Otonom
# VIDEO_PATH = 0 
VIDEO_PATH = "../B.mp4" 

FRAME_WIDTH, FRAME_HEIGHT = 640, 640

# Dictionary untuk mapping class ID ke nama
class_names = {0: 'green_ball', 1: 'red_ball'}

# List untuk menyimpan warna berdasarkan class ID
class_colors = [
    (0, 255, 0),  # Hijau untuk green_ball (ID 0)
    (0, 0, 255)   # Merah untuk red_ball (ID 1)
]

# Titik acuan frame
center_x_frame = FRAME_WIDTH // 2
left_x_frame = 100
right_x_frame = 540
pivot_point = (FRAME_WIDTH // 2, FRAME_HEIGHT)
bottom_pivot = 340
fps = 0.00

# Nilai RC (Kode 1)
rc1_value_awal = 1500
rc2_value_awal = 1500
rc3_value_awal = 1000
rc4_value_awal = 1500
rc3_value_lurus = 1500
rc1_value_kiri_mentok = 1000
rc1_value_kanan_mentok = 2000
rc1_value_kiri_sitik = 1300
rc1_value_kanan_sitik = 1800

# --- LOAD MODEL YOLO (Kode 1) ---
print("Memuat model YOLO...")
try:
    model = YOLO(MODEL_PATH)
    print("Model YOLO berhasil dimuat.")
except Exception as e:
    print(f"Gagal memuat model YOLO: {e}")
    exit()

# ==============================================================================
# === 3. FUNGSI HELPER (DARI KODE 1) ===
# ==============================================================================

def override_rc_channels(connection, rc1_value, rc2_value, rc3_value, rc4_value):
    """Mengirimkan perintah COMMAND_LONG untuk override channel RC1-RC4."""
    if connection is None:
        # print(f"Mode simulasi: RC1={rc1_value}, RC3={rc3_value}") # Uncomment untuk debug
        return  # Jangan lakukan apa-apa jika tidak terhubung

    try:
        connection.mav.rc_channels_override_send(
            connection.target_system,
            connection.target_component,
            rc1_value, rc2_value, rc3_value, rc4_value,
            0, 0, 0, 0
        )
    except Exception as e:
        print(f"Failed to override RC channels: {e}")

def get_vehicle_data():
    """Mendapatkan data telemetri dari vehicle (non-blocking)."""
    if connection is None:
        return None # Mode simulasi

    try:
        # [MODIFIKASI PENTING]
        # Menggunakan blocking=False atau timeout kecil
        # agar tidak mem-pause loop utama otonom.
        
        msg_gps = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=False, timeout=0.01)
        msg_att = connection.recv_match(type='ATTITUDE', blocking=False, timeout=0.01)
        msg_bat = connection.recv_match(type='SYS_STATUS', blocking=False, timeout=0.01)
        msg_vfr = connection.recv_match(type='VFR_HUD', blocking=False, timeout=0.01)
        msg_rc = connection.recv_match(type='RC_CHANNELS', blocking=False, timeout=0.01)
        msg_servo = connection.recv_match(type='SERVO_OUTPUT_RAW', blocking=False, timeout=0.01)
        msg_heartbeat = connection.recv_match(type='HEARTBEAT', blocking=False, timeout=0.01)

        # Jika pesan penting tidak ada, kembalikan None
        if not all([msg_gps, msg_att, msg_bat, msg_vfr, msg_rc, msg_heartbeat]):
            # print("Data MAVLink tidak lengkap di iterasi ini.")
            return None

        # Mengambil data GPS
        gps = {
            'lat': msg_gps.lat / 1e7,
            'lon': msg_gps.lon / 1e7,
            'alt': msg_gps.alt / 1e3,
            'cog': msg_gps.hdg / 100.0
        }

        # Mengambil data status baterai
        batt_status = {
            'voltage': msg_bat.voltage_battery / 1000.0,
            'current': msg_bat.current_battery / 100.0,
            'level': msg_bat.battery_remaining
        }

        # Mengambil data attitude
        attitude = {
            'roll': math.degrees(msg_att.roll),
            'pitch': math.degrees(msg_att.pitch),
            'yaw': math.degrees(msg_att.yaw)
        }

        # Mengambil data kecepatan
        speed = {
            'ground_speed': msg_vfr.groundspeed,
            'kmh': msg_vfr.groundspeed * 3.6,
            'knot': msg_vfr.groundspeed * 1.94384
        }
        
        heading = msg_vfr.heading
        baro = msg_vfr.alt

        # Mengambil data RC channel
        rc_channels = {
            'rc1': msg_rc.chan1_raw, 'rc2': msg_rc.chan2_raw,
            'rc3': msg_rc.chan3_raw, 'rc4': msg_rc.chan4_raw,
            'rc5': msg_rc.chan5_raw, 'rc6': msg_rc.chan6_raw
        }

        # Mengambil nilai servo output
        servo_output = {}
        if msg_servo:
            servo_output = {
                'steer': msg_servo.servo1_raw, 'th_mid': msg_servo.servo3_raw,
                'th_left': msg_servo.servo5_raw, 'th_right': msg_servo.servo7_raw
            }

        mode = mavutil.mode_string_v10(msg_heartbeat)
        is_armed = msg_heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED > 0
        is_armable = (
            mode in ["GUIDED", "AUTO", "STABILIZE"] and
            batt_status['level'] > 20 and
            msg_bat.current_battery >= 0
        )

        # Mengembalikan semua data dalam satu dictionary
        return {
            "gps": gps, "bat_status": batt_status, "heading": heading,
            "speed": speed, "baro": baro, "attitude": attitude,
            "rc_channels": rc_channels, "servo_output": servo_output,
            "mode": mode, "is_armed": is_armed, "is_armable": is_armable,
        }
        
    except Exception as e:
        print(f"Error di get_vehicle_data: {e}")
        return None

def draw_center_line(frame, fps, green, red, mid):
    """Fungsi untuk menggambar garis tengah berwarna putih pada frame."""
    cv2.putText(frame, f'FPS: {fps:.2f}', (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

    if green is not None:
        green_center_x, green_center_y = green[4]
        cv2.putText(frame, f'Green: ({green_center_x:.0f}, {green_center_y:.0f})', (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    if red is not None:
        red_center_x, red_center_y = red[4]
        cv2.putText(frame, f'Red: ({red_center_x:.0f}, {red_center_y:.0f})', (10, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    cv2.putText(frame, f'Mid: ({mid[0]:.0f}, {mid[1]:.0f})', (10, 100),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

    # Garis bantu
    cv2.line(frame, (center_x_frame, 0), (center_x_frame, FRAME_HEIGHT), (255, 255, 255), 2)
    cv2.line(frame, (left_x_frame, 0), (left_x_frame, FRAME_HEIGHT), (255, 255, 255), 2)
    cv2.line(frame, (right_x_frame, 0), (right_x_frame, FRAME_HEIGHT), (255, 255, 255), 2)
    cv2.line(frame, (0, bottom_pivot), (FRAME_WIDTH, bottom_pivot), (255, 255, 255), 2)

def calculate_distance(point1, point2):
    """Fungsi untuk menghitung jarak Euclidean antara dua titik."""
    return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

def calculate_mid_point(point1, point2):
    """Fungsi untuk menghitung titik tengah antara dua titik."""
    return ((point1[0] + point2[0]) / 2, (point1[1] + point2[1]) / 2)

# ==============================================================================
# === 4. FUNGSI HELPER (DARI KODE 2) ===
# ==============================================================================

def upload_opencv_frame_to_cloudinary(frame):
    """Upload frame OpenCV ke Cloudinary."""
    try:
        # print("\n[UPLOAD] Mengonversi frame OpenCV BGR ke RGB...")
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(frame_rgb)
        img.thumbnail((800, 800))
        buffer = io.BytesIO()
        img.save(buffer, format="JPEG", quality=80)
        buffer.seek(0)
        # print("[UPLOAD] Meng-upload gambar terkompresi ke Cloudinary...")
        hasil_upload = cloudinary.uploader.upload(buffer, folder="asv_lomba")
        url = hasil_upload.get('secure_url')
        print(f"[UPLOAD] Upload sukses. URL: {url}")
        return url
    except Exception as e:
        print(f"[UPLOAD] GAGAL meng-upload frame OpenCV: {e}")
        return None

def archive_and_reset_data():
    """Mengarsipkan data lama di Firebase dan mereset dashboard."""
    if ref is None:
        print("[INIT] Firebase tidak terhubung, skip arsip.")
        return
        
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

def draw_geotag_on_image(frame, sensor_data):
    """Menggambar info geotag ke frame OpenCV."""
    # print("[GEOTAG] Menambahkan geotag ke frame...")

    # Ambil data sensor saat ini
    # [MODIFIKASI] Sesuaikan key dengan struktur data dari get_vehicle_data
    sog_ms = sensor_data.get("speed", {}).get("ground_speed", 0.0)
    sog_knots = sog_ms * 1.94384
    sog_kmh = sog_ms * 3.6
    cog = sensor_data.get("gps", {}).get("cog", 0.0)
    lat = sensor_data.get("gps", {}).get("lat", 0.0)
    lon = sensor_data.get("gps", {}).get("lon", 0.0)

    now = datetime.now()
    day_str = f"Day: {now.strftime('%a')}"
    date_str = f"Date: {now.strftime('%d/%m/%Y')}"
    time_str = f"Time: {now.strftime('%H:%M:%S')}"
    
    lat_dir = "N" if lat >= 0 else "S"
    lon_dir = "E" if lon >= 0 else "W"
    coord_str = f"Coord: [{lat_dir} {abs(lat):.5f} {lon_dir} {abs(lon):.5f}]"
    
    sog_str = f"SOG: {sog_knots:.1f} knot ({sog_kmh:.1f} km/h)"
    cog_str = f"COG: {cog:.1f} deg"
    
    text_lines = [day_str, date_str, time_str, coord_str, sog_str, cog_str]
    
    y, x = 80, 30
    line_spacing = 30
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.7
    font_color = (255, 255, 255)
    bg_color = (0, 0, 0)
    thickness = 2
    
    for line in text_lines:
        cv2.putText(frame, line, (x + 1, y + 1), font, font_scale, bg_color, thickness)
        cv2.putText(frame, line, (x, y), font, font_scale, font_color, thickness)
        y += line_spacing
        
    return frame

# ==============================================================================
# === 5. LOGIKA THREADING (BARU) ===
# ==============================================================================

# Variabel global untuk berbagi data antar thread
shared_data = {
    "telemetry": None,          # Diisi oleh Main Thread
    "monitoring_frame": None,   # Diisi oleh Main Thread
    "capture_trigger": None     # Diisi oleh Main Thread (dari keyboard)
}
# Lock untuk melindungi shared_data
data_lock = threading.Lock()

# Data payload untuk Firebase (dari Kode 2)
data_payload = {
    "position_log": {"preparation": "In Progress", "start": "Pending", "floating_ball": 0, "surface_imaging": "Pending", "underwater_imaging": "Pending", "finish": "Pending"},
    "attitude": {"sog": 0.0, "cog": 0.0, "heading": 0.0}, "local_position": {"x": 0.0, "y": 0.0},
    "gps_location": {"lat": 0.0, "lon": 0.0}, "current_mission": "Preparation",
    "mission_images": {"surface": None, "underwater": None}, "track_id": "A",
    "race_start_timestamp": None, "race_finish_timestamp": None,
    "indicators": {"battery": 100, "last_update": None}
}

def monitoring_thread_task():
    """
    Fungsi ini berjalan di thread terpisah.
    Ini adalah isi dari 'while' loop Kode 2 (Monitoring).
    """
    global data_payload # Gunakan payload global
    print("[Worker Thread] Siap. Menunggu data telemetri...")

    while True:
        # Ambil salinan data terbaru dari Main Thread dengan aman
        with data_lock:
            current_telemetry = shared_data["telemetry"]
            current_frame = shared_data["monitoring_frame"]
            trigger = shared_data["capture_trigger"]
            shared_data["capture_trigger"] = None  # Reset trigger setelah diambil

        if ref is None:
            # print("[Worker Thread] Firebase tidak terhubung. Skipping loop.")
            time.sleep(1.0)
            continue

        # --- A. PROSES PENGIRIMAN TELEMETRI (SETIAP 1 DETIK) ---
        if current_telemetry:
            try:
                # Isi payload dari data telemetri terbaru
                data_payload["gps_location"]["lat"] = current_telemetry["gps"]['lat']
                data_payload["gps_location"]["lon"] = current_telemetry["gps"]['lon']
                data_payload["attitude"]["sog"] = current_telemetry["speed"]['ground_speed']
                data_payload["attitude"]["cog"] = current_telemetry["gps"]['cog']
                data_payload["attitude"]["heading"] = current_telemetry["heading"]
                data_payload["indicators"]["battery"] = current_telemetry["bat_status"]['level']
                data_payload["mode"] = current_telemetry["mode"] # Contoh tambah data

                # --- B. PROSES CAPTURE JIKA ADA TRIGGER ---
                if trigger and current_frame is not None:
                    print(f"[Worker Thread] Memproses trigger: {trigger}...")
                    log_key = f"{trigger}_imaging"
                    data_payload["position_log"][log_key] = "In Progress"
                    
                    # 1. Geotag (menggunakan data telemetri yang sama)
                    geotagged_frame = draw_geotag_on_image(current_frame.copy(), current_telemetry)
                    
                    # 2. Upload (Operasi Lambat)
                    url = upload_opencv_frame_to_cloudinary(geotagged_frame)
                    
                    if url:
                        data_payload["mission_images"][trigger] = url
                        data_payload["position_log"][log_key] = "Done"
                    else:
                        data_payload["position_log"][log_key] = "Failed"

                # --- C. KIRIM KE FIREBASE (Operasi Lambat) ---
                data_payload["indicators"]["last_update"] = time.time()
                ref.set(data_payload)
                print(f"[Worker Thread] Data dikirim. Baterai: {data_payload['indicators']['battery']}%", end='\r')

            except Exception as e:
                print(f"[Worker Thread] Error: {e}")
        
        # Tunggu 1 detik sebelum iterasi berikutnya
        # Ini adalah "gerbang 1 detik" dari Kode 2
        time.sleep(1.0)

# ==============================================================================
# === 6. FUNGSI MAIN LOOP (OTONOM - KODE 1) ===
# ==============================================================================

def main_loop():
    """
    Ini adalah LOGIKA KODE 1.
    Berjalan di Main Thread secepat mungkin (FPS tinggi).
    """
    
    # [PERHATIAN] Ganti 0 dan 1 sesuai dengan port kamera Anda
    print(f"Membuka kamera Otonom ({VIDEO_PATH})...")
    cap_otonom = cv2.VideoCapture(VIDEO_PATH)    # Kamera untuk YOLO
    print("Membuka kamera Monitoring (Port 1)...")
    cap_monitor = cv2.VideoCapture(1)            # Kamera untuk Geotag/Monitoring
    
    if not cap_otonom.isOpened():
        print("Error: Tidak bisa buka kamera Otonom.")
        return
    if not cap_monitor.isOpened():
        print("Error: Tidak bisa buka kamera Monitor.")
        return

    print("[Main Thread] Loop otonom dimulai...")
    
    mid_point = (320, 320)  # Nilai default
    
    # Data telemetri terakhir yang valid
    last_valid_telemetry = None 

    try:
        while True:
            # --- 1. BACA INPUT (Cepat) ---
            ret_o, frame_otonom = cap_otonom.read()
            ret_m, frame_monitor = cap_monitor.read()
            
            if not ret_o:
                print("[Main Thread] Video Otonom selesai atau gagal. Loop berhenti.")
                break
            if not ret_m:
                print("[Main Thread] Gagal baca frame Monitor.")
                # Tetap lanjutkan meski kamera monitor gagal
                frame_monitor = np.zeros((480, 640, 3), dtype=np.uint8) # Frame hitam

            # Resize frame otonom untuk YOLO
            frame_otonom_resized = cv2.resize(frame_otonom, (FRAME_WIDTH, FRAME_HEIGHT))

            # --- 2. DAPATKAN DATA TELEMETRI (Cepat) ---
            vehicle_data = get_vehicle_data()
            if vehicle_data:
                last_valid_telemetry = vehicle_data # Simpan data valid terakhir

            # --- 3. LOGIKA OTONOM (Kode 1) (Cepat) ---
            start_time = time.time()
            results = model.predict(source=frame_otonom_resized, imgsz=640, conf=0.6, iou=0.7, max_det=8, device='0', verbose=False)
            elapsed_time = time.time() - start_time
            fps = 1 / elapsed_time

            closest_green_ball = None
            closest_red_ball = None
            closest_green_distance = float('inf')
            closest_red_distance = float('inf')

            for result in results:
                for box in result.boxes:
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    conf = box.conf[0].item()
                    cls = int(box.cls[0].item())
                    
                    center_x_box = (x1 + x2) / 2
                    center_y_box = (y1 + y2) / 2
                    center_box = (center_x_box, center_y_box)
                    color = class_colors[cls] if cls < len(class_colors) else (255, 255, 255)
                    
                    distance_to_pivot = calculate_distance(center_box, pivot_point)

                    if center_y_box < bottom_pivot:
                        if cls == 0 and distance_to_pivot < closest_green_distance:
                            closest_green_distance = distance_to_pivot
                            closest_green_ball = (x1, y1, x2, y2, center_box, color)
                        if cls == 1 and distance_to_pivot < closest_red_distance:
                            closest_red_distance = distance_to_pivot
                            closest_red_ball = (x1, y1, x2, y2, center_box, color)
                    
                    # Gambar box (logika asli dari Kode 1)
                    cv2.rectangle(frame_otonom_resized, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
                    label = f'{class_names.get(cls, "Unknown")} {conf:.2f}'
                    cv2.putText(frame_otonom_resized, label, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            if closest_green_ball and closest_red_ball:
                _, _, _, _, green_center, _ = closest_green_ball
                _, _, _, _, red_center, _ = closest_red_ball
                
                mid_point = calculate_mid_point(green_center, red_center)
                
                # Gambar visualisasi (logika asli dari Kode 1)
                cv2.circle(frame_otonom_resized, (int(green_center[0]), int(green_center[1])), 5, (0, 255, 0), -1)
                cv2.circle(frame_otonom_resized, (int(red_center[0]), int(red_center[1])), 5, (0, 0, 255), -1)
                cv2.line(frame_otonom_resized, (int(green_center[0]), int(green_center[1])), (int(red_center[0]), int(red_center[1])), (0, 255, 255), 2)
                cv2.line(frame_otonom_resized, (int(mid_point[0]), int(mid_point[1])), pivot_point, (0, 255, 255), 2)
                cv2.circle(frame_otonom_resized, (int(mid_point[0]), int(mid_point[1])), 5, (255, 0, 0), -1)

            draw_center_line(frame_otonom_resized, fps, closest_green_ball, closest_red_ball, mid_point)

            # --- 4. LOGIKA KENDALI (Kode 1) (Cepat) ---
            if mid_point[0] > 250 and mid_point[0] < 390:
                override_rc_channels(connection, rc1_value_awal, rc2_value_awal, rc3_value_lurus, rc4_value_awal)
            elif mid_point[0] > 100 and mid_point[0] < 250:
                override_rc_channels(connection, rc1_value_kiri_sitik, rc2_value_awal, rc3_value_lurus, rc4_value_awal)
            elif mid_point[0] > 0 and mid_point[0] < 100:
                override_rc_channels(connection, rc1_value_kiri_mentok, rc2_value_awal, rc3_value_lurus, rc4_value_awal)
            elif mid_point[0] > 390 and mid_point[0] < 540:
                override_rc_channels(connection, rc1_value_kanan_sitik, rc2_value_awal, rc3_value_lurus, rc4_value_awal)
            elif mid_point[0] > 540 and mid_point[0] < 640:
                override_rc_channels(connection, rc1_value_kanan_mentok, rc2_value_awal, rc3_value_lurus, rc4_value_awal)
            
            # --- 5. [THREADING] BERBAGI DATA KE WORKER THREAD (Sangat Cepat) ---
            with data_lock:
                if last_valid_telemetry:
                    shared_data["telemetry"] = last_valid_telemetry
                shared_data["monitoring_frame"] = frame_monitor # Bagikan frame dari kamera monitor

            # --- 6. TAMPILKAN VISUAL & CEK KEYBOARD (Cepat) ---
            cv2.imshow('Otonom (Kamera Otonom)', frame_otonom_resized)
            cv2.imshow('Monitoring (Kamera Monitor)', frame_monitor)
            
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
        if connection:
            connection.close()
        print("[Main Thread] Selesai.")

# ==============================================================================
# === 7. EKSEKUSI PROGRAM UTAMA ===
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

    print("Program Selesai.")
