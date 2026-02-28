from dronekit import connect, VehicleMode, LocationGlobalRelative
import argparse
import geopy.distance
import time
import firebase_admin
from firebase_admin import db, credentials
import RPi.GPIO as GPIO

# ==============================================================
# 🔥 FIREBASE SETUP
# ==============================================================

cred = credentials.Certificate("dronecred.json")
firebase_admin.initialize_app(cred, {
    "databaseURL": "https://drone-delights-default-rtdb.firebaseio.com/"
})

order_ref = db.reference("orders/current_order")

# ==============================================================
# 🚁 DRONE CONNECTION
# ==============================================================

def connectMyCopter():
    parser = argparse.ArgumentParser()
    parser.add_argument('--connect')
    args = parser.parse_args()
    return connect(args.connect, baud=57600, wait_ready=True)

vehicle = connectMyCopter()

# ==============================================================
# 🏠 HOME LOCATION
# ==============================================================

HOME_LAT = 16.565980
HOME_LON = 81.521722
FLIGHT_ALT = 30

# ==============================================================
# 🟦 SERVO SETUP (Raspberry Pi GPIO18)
# ==============================================================

SERVO_PIN = 18        # GPIO18 (Pin 12)

GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PIN, GPIO.OUT)

pwm = GPIO.PWM(SERVO_PIN, 50)  # 50Hz PWM
pwm.start(0)

def angle_to_duty(angle):
    return 2 + (angle / 18)

def rotate_servo_twice():
    print("🔄 Servo rotating 2 cycles...")

    # Cycle 1
    pwm.ChangeDutyCycle(angle_to_duty(180))
    time.sleep(0.5)
    pwm.ChangeDutyCycle(angle_to_duty(0))
    time.sleep(0.5)

    # Cycle 2
    pwm.ChangeDutyCycle(angle_to_duty(180))
    time.sleep(0.5)
    pwm.ChangeDutyCycle(angle_to_duty(0))
    time.sleep(0.5)

    pwm.ChangeDutyCycle(0)
    print("✔ Servo rotation complete")

# ==============================================================
# HELPER FUNCTIONS
# ==============================================================

def get_distance(cord1, cord2):
    return geopy.distance.geodesic(cord1, cord2).meters


def arm_and_takeoff(target_alt):

    print("Pre-arm checks...")
    while not vehicle.is_armable:
        time.sleep(1)

    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        time.sleep(1)

    print("Taking off...")
    vehicle.simple_takeoff(target_alt)

    while vehicle.location.global_relative_frame.alt < target_alt * 0.95:
        print(" Alt:", vehicle.location.global_relative_frame.alt)
        time.sleep(1)

    print("🚀 Takeoff Complete")


def goto_location(lat, lon, speed=6):

    target = LocationGlobalRelative(lat, lon, FLIGHT_ALT)
    vehicle.simple_goto(target, groundspeed=speed)

    while True:
        current = (
            vehicle.location.global_relative_frame.lat,
            vehicle.location.global_relative_frame.lon
        )

        distance = get_distance(current, (lat, lon))
        print("Distance to target:", distance)

        if distance <= 5:
            print("📍 Reached Location")
            break

        time.sleep(1)

# ==============================================================
# 🟡 WAIT FOR WEBSITE ORDER
# ==============================================================

print("🟡 Waiting for order...")

while True:
    order = order_ref.get()

    if order and order.get("drone_status") == "Not Dispatched":
        print("🟢 Order Received")
        break

    time.sleep(2)

# ==============================================================
# 📍 GET DYNAMIC SHOP COORDS
# ==============================================================

shop_lat = float(order.get("shop_lat"))
shop_lng = float(order.get("shop_lng"))

order_ref.update({"drone_status": "Taking Off"})

# ==============================================================
# 🚀 TAKEOFF FROM HOME
# ==============================================================

arm_and_takeoff(FLIGHT_ALT)

# ==============================================================
# 🏪 GO TO SHOP
# ==============================================================

print("🏪 Flying to Shop...")
goto_location(shop_lat, shop_lng)

print("🛬 Landing at shop...")
vehicle.mode = VehicleMode("LAND")

while vehicle.location.global_relative_frame.alt > 0.2:
    time.sleep(1)

# ------ AFTER LANDING SERVO WORK ------
print("⏳ Waiting 5 sec before servo...")
time.sleep(5)

rotate_servo_twice()

print("⏳ Waiting 5 sec before takeoff...")
time.sleep(5)

# ==============================================================
# 🚀 TAKEOFF AGAIN FROM SHOP
# ==============================================================

vehicle.mode = VehicleMode("GUIDED")
time.sleep(2)
arm_and_takeoff(FLIGHT_ALT)

# ==============================================================
# 🏠 GO TO CUSTOMER LOCATION
# ==============================================================

customer_lat = float(order.get("hostel_lat"))
customer_lon = float(order.get("hostel_lng"))

print("🏠 Flying to Customer...")
goto_location(customer_lat, customer_lon)

# ==============================================================
# 🔐 OTP WAIT
# ==============================================================

order_ref.update({"drone_status": "Drone Reached"})
print("📡 Waiting up to 60s for OTP...")

otp_timeout = 60
start_time = time.time()
otp_verified = False

while True:
    status = order_ref.child("drone_status").get()

    if status == "Land":
        otp_verified = True
        break

    if time.time() - start_time > otp_timeout:
        break

    time.sleep(1)

# ==============================================================
# DECISION: LAND OR RETURN
# ==============================================================

if otp_verified:
    print("🛬 Landing at customer...")
    vehicle.mode = VehicleMode("LAND")

    while vehicle.location.global_relative_frame.alt > 0.2:
        time.sleep(1)

    # ---- AFTER LANDING SERVO ----
    print("⏳ Waiting 5 sec before servo...")
    time.sleep(5)

    rotate_servo_twice()

    print("⏳ Waiting 5 sec more...")
    time.sleep(5)

    print("⏳ Final 30 sec delivery wait...")
    time.sleep(30)

else:
    print("❌ OTP failed or timeout. Returning home.")

# ==============================================================
# 🔁 RETURN TO HOME
# ==============================================================

print("🚁 Returning to Home...")
vehicle.mode = VehicleMode("GUIDED")
time.sleep(2)

arm_and_takeoff(FLIGHT_ALT)

goto_location(HOME_LAT, HOME_LON)

vehicle.mode = VehicleMode("LAND")
while vehicle.location.global_relative_frame.alt > 0.2:
    time.sleep(1)

order_ref.update({"drone_status": "Mission Completed"})

print("🏠 Mission Completed Successfully")

# CLEANUP
pwm.stop()
GPIO.cleanup()
vehicle.close()
