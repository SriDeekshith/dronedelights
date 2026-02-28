from dronekit import connect, VehicleMode, LocationGlobalRelative
import argparse
import geopy.distance
import time
import firebase_admin
from firebase_admin import db, credentials

# ==============================================================
# 🔥 FIREBASE SETUP (MUST MATCH WEBSITE DATABASE)
# ==============================================================

cred = credentials.Certificate("dronecred.json")
firebase_admin.initialize_app(cred, {
    "databaseURL": "https://drone-delights-default-rtdb.firebaseio.com/"
})

order_ref = db.reference("orders/current_order")

# ==============================================================
# 🚁 CONNECT TO DRONE
# ==============================================================

def connectMyCopter():
    parser = argparse.ArgumentParser()
    parser.add_argument('--connect')
    args = parser.parse_args()
    return connect(args.connect, baud=57600, wait_ready=True)

vehicle = connectMyCopter()

# ==============================================================
# 🏠 HOME LOCATION (DRONE CENTRE)
# ==============================================================

HOME_LAT = 16.565980
HOME_LON = 81.521722
FLIGHT_ALT = 30

# ==============================================================
# 📍 SHOP LOCATION (FIXED IN DRONE CODE)
# ==============================================================

SHOP_LAT = 16.565917
SHOP_LON = 81.521688

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

        if distance <= 3:
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

# Update status
order_ref.update({"drone_status": "Taking Off"})

# ==============================================================
# 🚀 TAKEOFF FROM HOME
# ==============================================================

arm_and_takeoff(FLIGHT_ALT)

# ==============================================================
# 🏪 GO TO SHOP
# ==============================================================

print("🏪 Flying to Shop...")
goto_location(SHOP_LAT, SHOP_LON)

print("🛬 Landing at shop...")
vehicle.mode = VehicleMode("LAND")

while vehicle.location.global_relative_frame.alt > 0.2:
    print(" Alt:", vehicle.location.global_relative_frame.alt)
    time.sleep(1)

print("✔ Landed at shop. Waiting 60 seconds...")
time.sleep(60)


# ==============================================================
# 🚀 TAKEOFF AGAIN FROM SHOP
# ==============================================================

print("🚀 Taking off from shop...")
vehicle.mode = VehicleMode("GUIDED")
time.sleep(2)

arm_and_takeoff(FLIGHT_ALT)

# ==============================================================
# 🏠 GO TO CUSTOMER LOCATION (FROM WEBSITE)
# ==============================================================

customer_lat = float(order.get("hostel_lat"))
customer_lon = float(order.get("hostel_lng"))

print("🏠 Flying to Customer...")
goto_location(customer_lat, customer_lon)

# ==============================================================
# 🔐 HOVER & WAIT FOR OTP (1 MINUTE MAX)
# ==============================================================

order_ref.update({"drone_status": "Drone Reached"})
print("📡 Waiting for OTP (1 minute timeout)...")

otp_timeout = 60
start_time = time.time()
otp_verified = False

while True:
    status = order_ref.child("drone_status").get()

    if status == "Land":
        print("✅ OTP Verified")
        otp_verified = True
        break

    elapsed = time.time() - start_time
    if elapsed > otp_timeout:
        print("⛔ OTP Timeout → Returning Home")
        order_ref.update({"drone_status": "OTP Timeout - Returning"})
        break

    time.sleep(2)

# ==============================================================
# DECISION: LAND OR RETURN
# ==============================================================

if otp_verified:

    print("🛬 Landing at customer location...")
    vehicle.mode = VehicleMode("LAND")

    while vehicle.location.global_relative_frame.alt > 0.2:
        print(" Alt:", vehicle.location.global_relative_frame.alt)
        time.sleep(1)

    print("✔ Landed. Waiting 30 seconds...")
    time.sleep(30)

else:
    print("❌ Skipping landing, returning immediately.")


# ==============================================================
# 🔁 RETURN TO HOME
# ==============================================================

print("🚁 Returning to Home...")

vehicle.mode = VehicleMode("GUIDED")
time.sleep(2)

arm_and_takeoff(FLIGHT_ALT)

goto_location(HOME_LAT, HOME_LON)

print("🛬 Landing at Home...")
vehicle.mode = VehicleMode("LAND")

while vehicle.location.global_relative_frame.alt > 0.2:
    print(" Alt:", vehicle.location.global_relative_frame.alt)
    time.sleep(1)

print("🏠 Mission Completed Successfully")

order_ref.update({"drone_status": "Mission Completed"})
vehicle.close()
