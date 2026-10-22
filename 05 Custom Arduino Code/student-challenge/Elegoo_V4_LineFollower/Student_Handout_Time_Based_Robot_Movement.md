# Student Handout: How the Robot Car Moves

**Name:** ____________________________________  **Date:** __________________

## The main idea: command, wait, change

The car is controlled by two motors: one on the **left** and one on the **right**.
The program sets each motor's direction and power, waits for a chosen amount of
time, and then gives the motors a new command or stops them.

```text
set motor directions and speeds
wait (delay) for a measured time
set a new movement or stop
```

For example, `moveForward(100); delay(1000);` tells both wheels to move forward
at power 100, then keeps that command for 1,000 milliseconds (1 second). The
next instruction can change the movement. A `delay()` does not steer or stop the
car by itself; the motor command remains in effect while the program waits.

In the motor test, the program uses this **time-based** approach. In the line
follower, the program also reads sensors and chooses a motor command repeatedly.
The car's actual distance and turning angle can vary with battery level, floor
surface, wheel grip, and the car's starting position.

## Motor power and direction

- Motor power is set with a number from **0 to 255**. A larger number requests
  more motor power; `0` turns that motor off.
- Each wheel also has a direction: forward or reverse.
- Different speeds on the two wheels make the car curve. Opposite directions
  make it pivot or spin.
- These power values are not a guaranteed speed in centimeters per second.

## Movement functions in this sketch

| Function | What it commands |
|---|---|
| `moveForward(speed)` | Both wheels forward at the same power. |
| `moveReverse(speed)` | Both wheels backward at the same power. |
| `turnLeft(outerSpeed, innerSpeed)` | Both wheels forward. The right wheel (outside wheel) uses `outerSpeed`; the left wheel (inside wheel) uses `innerSpeed`. A slower inside wheel curves the car left. If the speeds are equal, it goes straight rather than turning. |
| `turnRight(outerSpeed, innerSpeed)` | Both wheels forward. The left wheel (outside wheel) uses `outerSpeed`; the right wheel (inside wheel) uses `innerSpeed`. A slower inside wheel curves the car right. If the speeds are equal, it goes straight rather than turning. |
| `turnLeft(speed)` | Left wheel backward and right wheel forward at the same power: pivot left. |
| `turnRight(speed)` | Left wheel forward and right wheel backward at the same power: pivot right. |
| `spinLeft(speed)` | Left wheel backward and right wheel forward: spin left. |
| `spinRight(speed)` | Left wheel forward and right wheel backward: spin right. |
| `stopMotors()` | Sets both wheel powers to zero. |

This sketch defines both one-argument and two-argument versions of `turnLeft`
and `turnRight`. The one-argument versions pivot; the two-argument versions
curve by setting different wheel powers. `spinLeft` and `spinRight` also pivot
with opposite wheel directions.

## What happens in the motor test?

After the car is powered on, the program first waits for the MODE button so it
can show live line-sensor readings. After that button is released,
`motorTest()` waits for the MODE button **again**. It then runs this sequence:

| Order | Command | Time | Expected movement |
|---:|---|---:|---|
| 1 | `moveForward(100)` | 1 second | Both wheels forward; car moves ahead. |
| 2 | `moveReverse(100)` | 1 second | Both wheels backward; car reverses. |
| 3 | `turnLeft(200, 200)` | 1 second | Both wheels forward at equal power; this is straight, not a left curve. |
| 4 | `turnRight(200, 200)` | 1 second | Both wheels forward at equal power; this is straight, not a right curve. |
| 5 | `stopMotors()` | 1 second | Both motor powers are zero. |
| 6 | `spinRight(100)` | 2 seconds | Wheels run in opposite directions; car spins right. |
| 7 | `spinLeft(100)` | 2 seconds | Wheels run in opposite directions; car spins left. |
| 8 | `stopMotors()` | No delay here | Both motor powers are set to zero. |

The durations come from the `delay(...)` instructions immediately after the
movement commands. The source comment says the right spin lasts 3 seconds, but
the actual delay is `2000` milliseconds, or **2 seconds**.

After the motor test, the program waits 1 second and starts line tracking.

## How line following makes decisions

The car has three line sensors: **left**, **middle**, and **right**. Each sensor
returns a reading from 0 to 1023. The sketch treats a reading **greater than
600** as detecting the black line.

Each pass through `loop()` reads the sensors, prints their readings, and waits
500 milliseconds. It then chooses a motor command:

| Sensor result | Program action |
|---|---|
| Only the middle sensor detects the line | `moveForward(130)` |
| The left sensor detects it and the right sensor does not | `spinLeft(140)` |
| The right sensor detects it and the left sensor does not | `spinRight(140)` |
| All three sensors detect it | `stopMotors()` |
| None of the sensors detect it | `moveForward(90)` to continue gently ahead |
| Any other combination | `moveForward(130)` |

The program keeps repeating this read–decide–command cycle. The 500 ms delay is
the pause in each loop; it is **not** a fixed duration for every movement. A new
sensor reading and decision can replace the current motor command on a later
loop.

## Two timing helpers

- `waitForStart()` waits until the MODE button is pressed, then waits another
  2 seconds before continuing.
- `stopTime(mS)` puts the motor driver into standby for `mS` milliseconds, then
  takes it out of standby. This is different from `stopMotors()`, which sets
  both motor powers to zero. `stopTime()` is defined in this sketch but is not
  used by the motor test or line-following loop.

## Try it: predict, test, explain

1. What do you expect if the program runs `moveForward(100); delay(2000);`?

   ___________________________________________________________________________

2. In `turnLeft(200, 200)`, why does the car not make a clear left curve?

   ___________________________________________________________________________

3. What is the difference between a timed motor-test command and a line-following
   command?

   ___________________________________________________________________________

4. Change one motor speed or one delay in the motor test. Predict what will
   happen, then test your prediction with the car safely on the floor.

   **My change:** ____________________________________________________________

   **My prediction:** ________________________________________________________

   **What happened:** ________________________________________________________

## Safety

Keep the car on the floor with its wheels clear of obstacles. Keep hands, hair,
and loose objects away from the wheels. Be ready to switch off the car if it
moves unexpectedly.

---

*Based on `Elegoo_V4_LineFollower.ino`. The handout describes what the sketch
commands; real movement can vary between cars and surfaces.*
