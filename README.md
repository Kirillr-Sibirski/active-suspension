# Table of Contents

[Abstract [1](#abstract)](#abstract)

[Introduction
[1](#this-essay-details-the-development-of-an-active-suspension-system-aimed-at-improving-the-safety-of-off-road-vehicles-by-addressing-the-challenge-of-rollovers-on-uneven-terrain.-through-an-in-depth-literature-review-and-analysis-of-vehicle-dynamics-the-project-identifies-an-approach-to-enhance-current-rollover-prevention-techniques.-utilizing-a-110th-scale-model-car-for-practical-and-cost-effective-experimentation-this-study-explores-the-integration-of-mechanical-electronic-and-software-engineering-to-adjust-the-vehicles-centre-of-mass-dynamically-ensuring-constant-wheel-ground-contact-for-better-traction.-while-the-system-demonstrated-potential-in-mitigating-specific-rollover-scenarios-its-effectiveness-across-all-conditions-highlighted-areas-for-further-development.-this-endeavour-emphasizes-the-value-of-a-multidisciplinary-approach-to-engineering-challenges-contributing-to-the-discussion-on-enhancing-off-road-vehicle-safety.-the-experience-gained-from-this-project-offers-insights-into-the-complexities-of-designing-safety-mechanisms-for-dynamic-environments.)](#this-essay-details-the-development-of-an-active-suspension-system-aimed-at-improving-the-safety-of-off-road-vehicles-by-addressing-the-challenge-of-rollovers-on-uneven-terrain.-through-an-in-depth-literature-review-and-analysis-of-vehicle-dynamics-the-project-identifies-an-approach-to-enhance-current-rollover-prevention-techniques.-utilizing-a-110th-scale-model-car-for-practical-and-cost-effective-experimentation-this-study-explores-the-integration-of-mechanical-electronic-and-software-engineering-to-adjust-the-vehicles-centre-of-mass-dynamically-ensuring-constant-wheel-ground-contact-for-better-traction.-while-the-system-demonstrated-potential-in-mitigating-specific-rollover-scenarios-its-effectiveness-across-all-conditions-highlighted-areas-for-further-development.-this-endeavour-emphasizes-the-value-of-a-multidisciplinary-approach-to-engineering-challenges-contributing-to-the-discussion-on-enhancing-off-road-vehicle-safety.-the-experience-gained-from-this-project-offers-insights-into-the-complexities-of-designing-safety-mechanisms-for-dynamic-environments.)

[Process [2](#process)](#process)

[1: Literature Review [2](#literature-review)](#literature-review)

[2: Theoretical Suspension Design
[4](#theoretical-suspension-design)](#theoretical-suspension-design)

[3: Initial Tests [5](#initial-tests)](#initial-tests)

[4: Mechanical System Prototyping & Development
[7](#mechanical-system-prototyping-development)](#mechanical-system-prototyping-development)

[5: Electronics Design [13](#electronics-design)](#electronics-design)

[6: Software [16](#software)](#software)

[7: Performance Assessment
[20](#performance-assessment)](#performance-assessment)

[Evaluation [21](#_Toc161216122)](#_Toc161216122)

[Bibliography [21](#_Toc161216123)](#_Toc161216123)

[Appendices [22](#appendices)](#appendices)

# Abstract

# This essay details the development of an active suspension system aimed at improving the safety of off-road vehicles by addressing the challenge of rollovers on uneven terrain. Through an in-depth literature review and analysis of vehicle dynamics, the project identifies an approach to enhance current rollover prevention techniques. Utilizing a 1/10th scale model car for practical and cost-effective experimentation, this study explores the integration of mechanical, electronic, and software engineering to adjust the vehicle's centre of mass dynamically, ensuring constant wheel-ground contact for better traction. While the system demonstrated potential in mitigating specific rollover scenarios, its effectiveness across all conditions highlighted areas for further development. This endeavour emphasizes the value of a multidisciplinary approach to engineering challenges, contributing to the discussion on enhancing off-road vehicle safety. The experience gained from this project offers insights into the complexities of designing safety mechanisms for dynamic environments.

# Introduction

The current state of off-road vehicles has gradually improved over the
past decades, however, it still encounters lots of challenges,
particularly related to maintaining stability when navigating sloping
terrains, where the possibility of a rollover poses a concern. This
project aims to engineer an active suspension system specifically
designed for smaller-scale off-road vehicles, with a primary focus on
minimizing rollover[^1] occurrences, particularly on slopes. This
project seeks to improve the off-road capabilities of a vehicle,
equipping it to navigate challenging terrain with higher stability and
reduced risk of a rollover. Additionally, such systems could potentially
be adapted for use on rovers exploring extra-terrestrial environments,
ensuring enhanced stability in diverse planetary terrains.

Throughout this project, my aim is to minimize specifically untripped
rollover[^2] (Wikipedia Foundation, 2024) as this event can be minimized
with an active suspension system which would change the centre of
mass[^3] of a vehicle, thus, increasing the required inertia to cause a
rollover.

The decision to use an active suspension system on a 1/10th scale remote
controlled off-road car[^4] allows a convenient implementation of all
the mechanical systems and electronics while providing
cost-effectiveness and simpler testing procedures compared to larger
scale models. Moreover, off-road vehicles, SUVs in particular, are more
prone to rollovers due to the raised suspensions which raises the centre
of mass.

# Process

## 1: Literature Review

According to (Wikipedia Foundation, 2022), current methods used by
National Highway Traffic Safety Administration (NHTSA) to test vehicle
susceptiveness to rollovers is a tilt test. The test evaluates the
vehicle's weight distribution and thus its centre of gravity
positioning. It involves tilting the vehicle laterally on a mobile
platform in a simulated side-to-side direction. To pass, the vehicle
must remain stable without tipping over until reaching a predetermined
tilt angle set by the testing protocol.

Some of the rollover prevention systems used in production vehicles
include: ARP (Active Rollover Prevention) that recognizes impending
rollover and selectively applies brakes to resist (Wikipedia Foundation,
2020); Gyroscope - a special device that tries to maintain orientational
or angular velocity (Wikipedia Foundation, 2023); Crosswind
Stabilization (CWS) - assists drivers in controlling a vehicle during
strong wind conditions (Wikipedia Foundation, 2023); The most common
method in use today are anti-roll bars in active, semi-active or passive
configurations, it is a suspension part that helps reduce the body
roll[^5] of a vehicle during fast cornering or over road irregularities.
It links opposite front or rear wheels to a spring. This increases the
suspension's roll stiffness — its resistance to roll in turns (Wikipedia
Foundation, 2023). Consequentially, none of the existing
production-level rollover prevention systems utilize active suspension
systems.

Principle of skyhook theory (Qazizadeh, 2017) states that an ideal
suspension system can uphold vehicle’s stable posture, unaffected to
weight shifts or uneven road surfaces, like an imaginary sky hook. This
is the main concept is key in preventing rollovers as in a rollover, a
vehicle is leaning to one side to such an extent that the weight of that
part of the vehicle dominates, leading it to tilt, consequently causing
it to tip over. By achieving perfect stability, or in other words,
preventing any body roll, the occurrence of a rollover would be averted.

An active suspension system is a technology in vehicles that adjusts the
vehicle's shock absorbers[^6] [^7] in real time using electronic sensors
and actuators, that can actively raise or lower the suspension and
actively counteract forces such as body roll, could be close to being a
‘skyhook’ by actively responding to road conditions. Pneumatic Active
Suspension (Li & Lee, 2019) is seemed to be used by most automakers as
per its convenience and lower cost, however, it is a slower reacting
system. Another type of an active suspension found in literature is
Electromagnetic damper (Pranav Teli, 2019). Unlike conventional
suspension systems that rely on typical dampers, the system incorporates
linear DC motors that modulate the suspension by adjusting its
stiffness. While this technology has not been integrated into
mass-produced vehicles, Bose and other automakers (NIO, 2024) have
presented a proof of concept (Howard, 2017).

As per results of (Westhuizen & Els, 2013), body roll can be minimized
with active suspension. The active suspension system used in the paper
is hydropneumatics and the amount of oil in each strut[^8] was
controlled adjusting the height of the tested vehicle which is a slower
reacting suspension system and is not likely to be functional at higher
speeds.

This project was partially inspired by the works of (Cockfield, 2021)
who has created an active suspension similar to what was done in this
project; however, his aim was to enhance vehicle’s steering control and
overall driving experience rather than specifically preventing
rollovers.

## 2: Theoretical Suspension Design

The sprung mass refers to the parts that the suspension holds up, like
the body and chassis in the middle of the car. On the other hand, the
unsprung mass includes parts not supported by the suspension, such as
the dampers, control arms, and wheels \[10\]. When the suspension
compresses, it primarily affects the sprung mass, while the rebound
phase controls the unsprung mass.

The objective of a damper is to mitigate the impact of the frequencies
associated with the sprung[^9] and unsprung[^10] masses (Valor Offroad
LLC, n.d.) when encountering diverse road surfaces. The higher amount
and the lower size of bumps, the higher the frequency oscillations. In
full-sized passenger cars, the natural frequency of the unsprung mass on
a smooth road typically ranges between 10 and 12 Hz. However, given the
relatively smaller size of the 1/10th model, the unsprung mass frequency
is expected to be higher. Attempting to use only an actuator to actively
reduce these higher frequencies poses a challenge, and currently, there
are no viable solutions identified for this issue.

Nonetheless, most active suspension systems employ a damper and a spring
configuration to mitigate these high-frequency oscillations, and an
actuator to manage the lower frequency oscillations. This design was
chosen which serves a dual purpose: it prevents potential damage to the
actuator and effectively absorbs abrupt terrain changes.
<img src="media/image1.png" style="width:3.37505in;height:4.17532in"
alt="A diagram of a mass motor Description automatically generated" />

## 3: Initial Tests

The hypothesis suggests that adjusting the height difference of the
shocks on difference sides (right/left or front/rear) to move the centre
of mass closer to the opposite direction of a tilt may increase the tilt
angle[^11]. This hypothesis is based on the expectation that such
adjustments could influence the vehicle's weight distribution and
balance, potentially enhancing its ability to tilt in response to
external forces or varying terrain. Exploring this hypothesis could help
me understand whether an active suspension system, which involves the
change in height difference between shock absorbers on either side,
would indeed prevent a rollover.

To examine this hypothesis, I constructed an apparatus integrated with
four shocks and an approximate 0.5 kg weight on top which emulates the
sprung mass of a vehicle in this experimental setup. The height
difference is adjusted using a nut on a screw which holds the shock
absorber. The experiment involves changing the height difference and
observing the effect it has on angle of the slope at which the apparatus
experiences is a rollover. The slippage of the apparatus was prevented
by gluing it to the slope on one side. The improvised slope was moved by
hand until the apparatus starts to tip over, that angle was then
measured with a protractor.

<img src="media/image2.jpeg" style="width:5in;height:3.75in" />

Here is the table of the average results that were obtained:

| **Height difference between two sides (cm)** | **Angle of a slide at which a rollover occurred** |
|----------------------------------------------|---------------------------------------------------|
| 0                                            | 40°                                               |
| 0.9                                          | 50°                                               |
| 2                                            | 53°                                               |
| 3.6                                          | 53°                                               |
| 4.9                                          | 55°                                               |
| 6.5                                          | 62°                                               |

<img src="media/image3.png" style="width:7.1288in;height:3.91192in"
alt="A graph with a line Description automatically generated" />

The graph was plotted from the results, and the best-fit line was drawn.
As a result, my hypothesis has been proven right because with an
increase in height difference, the angle of rollover occurrence also
increases proportionally at ≈ 3.4°/cm.

## 4: Mechanical System Prototyping & Development

My idea involved devising an alternative system similar to Bose's
innovation but using cost-effective, readily available components. The
initial plan encompassed employing a linear actuator[^12] to manage the
vertical adjustments of the dampers, ensuring optimal height control,
while a sufficiently robust DC motor would handle the lifting of the RC
model's weight. While the structure would fit suitably on the car body
shell’s[^13] elevated rear section, accommodating inside it, challenges
arose in integrating it into the front, where the hood is situated. To
resolve this, I devised a horizontal suspension system for the front.
The mechanism involved the transmission of force from uneven terrain to
a horizontal shock absorber via a triangular component, thereby enabling
the shock absorber to be fitted linear actuator to be placed
horizontally fitting under the body shell.

<img src="media/image4.png" style="width:5in;height:2.85417in" />

<img src="media/image5.png" style="width:6.9147in;height:2.74183in"
alt="A diagram of a machine Description automatically generated" />

As per my sketch, the linear actuator uses a threaded rod which spins
inside a nut attached to a plastic tube which is then attached to the
shock absorber. I then added four curved metal strips which move freely
with the plastic tube but prevent shock absorber from bending or
spinning where it is attached to the tube.

The linear actuator was attached to the back of the model car; however,
the system was slow even when spinning by hand which also required quite
a lot of force which made it impossible to use a motor of reasonable
size. I was also concerned that the height adjustment might not have a
significant enough difference to prevent rollover. Additionally, the
system's height and weight were already substantial, making it essential
to significantly reduce its size to accommodate it inside the model
car's shell body.

<img src="media/image6.jpeg" style="width:3.75in;height:5in" />

After conducting some more research, I discovered that a servo motor
(servo)[^14] would serve as an optimal solution, it is more compact
while having a larger torque. My initial concern revolved around servo’s
potential limited rotation distance, which resulted in the
implementation of a pivot to amplify its effect and reduce the force
necessary for vehicle elevation. The image below shows an experimental
setup utilizing the triangular structure as a pivot previously mentioned
in this chapter.

<img src="media/image7.png" style="width:5in;height:3.11458in" />

The results were promising — the shock exhibits the desired vertical
movement, and by utilizing a more potent servo motor (currently
operating at 6kg/cm), the range of motion can be further increased.

This revised system has greater compactness and speed compared to the
previous iteration employing linear actuators, allowing it to fit
beneath the car's shell.

Maximum ground clearance achieved when the servo was rotated to its
maximum was 98 mm. Minimum ground clearance achieved when the servo was
rotated to its minimum was 87 mm. Default ground clearance of the car:
93 mm. 12.6% height difference was achieved.

Further enhancements were required to reinforce the system's structure
and rigidity, so I opted to directly attach a servo motor to the shock
absorber using its extended servo horn[^15] which was made from a metal
sheet, eliminating the need for the triangular structure. This
modification has notably increased the system's compactness and improved
its performance. However, it now demands a higher torque from the servo
(in the experiment, 30kg/cm was used). Additionally, there's a concern
about the increased force applied to the servo, potentially leading to
faster wear on servo’s gearbox.

<img src="media/image8.jpeg" style="width:3.75in;height:5in" />

I made alterations to the design by connecting the shock absorber
directly to the servo horn without the extension, eliminating some
complexity. Even without the extension, the height difference of 47.1%
is satisfactory at 114 mm at max position and 76 mm at min position.
Additionally, I've installed all four servos onto the car, each
possessing 20kg/cm, slightly less than the ones used in the previous
experiment but still believed to be adequately robust.

<img src="media/image9.png" style="width:5in;height:4.71875in" />

The prototyped frames that were used to attach servo motors to the frame
of the car had a problem as they were made from perforated metal sheets
that were not capable of withstanding the required load and were bending
resulting in smaller active suspension travel (can be seen on the photo
above). The solution was to use a more rigid material such as plastic
thus ensuring structural integrity[^16]. Frames for servos were 3D
printed[^17] using PLA[^18] plastic and attached to the car frame. The
results were great, everything was rigid and felt quite strong, I was
also able to attach new body shell holders to each servo frame.

<img src="media/image10.png" style="width:6.26806in;height:4.70139in"
alt="A black machine with wires and wires Description automatically generated" />

## 5: Electronics Design

For testing the newly assembled mechanical system, I have integrated
electronics to control all four servos simultaneously with basic
commands. The setup includes an Arduino Nano[^19] board, x4 servo
motors[^20] a potentiometer for adjusting the suspension height and two
buttons for mode selection. All the electronics is powered from
computer’s USB port through the microcontroller except for the four
servo motors which require a higher current and therefore are powered by
a 7.4V LiPo battery cell[^21]. Everything was attached onto a
breadboard[^22] and connected with some jumper wires[^23]. Current modes
available include adjusting the height of all shocks simultaneously,
tilting to either side, and raising or lowering the front or back.

Simplified electronics diagram:

<img src="media/image11.png" style="width:6.76596in;height:3.42527in"
alt="A diagram of a computer network Description automatically generated with medium confidence" />

After testing the system and determining that it works fine (videos can
be found in the Appendices section), the buttons and the potentiometer
can be removed for the system to be made fully automated.

The key component of the automated system is the gyroscope module, the
accurate description of its workings can be found at
(LastMinuteEngineers.com, n.d.). It is used to determine the car’s
position in space. For this project, I am using MPU6050 model which is
widely available and can be easily connected to an Arduino board. The
module is directly powered from the microcontroller using 5V output and
is connected using I2C[^24] (SLT and SDA pins).

Moreover, for the system to be truly independent, it should be made
wireless removing the USB connection to the computer. In order achieve
that, the Arduino board is powered from the RC receiver[^25] which
provides a steady 4.8V of power.

Here is a diagram of the new electronics:

<img src="media/image12.png" style="width:7.02162in;height:4.23895in" />

All the electronics were later soldered on a perfboard[^26] which
prevented the issue of the signal getting jammed or unstable when using
breadboard and jumper wires. Additionally, an electronics enclosure was
3D printed using PLA plastic to prevent the electronics from being
damaged with dirt or dust.

<img src="media/image13.png" style="width:3.54659in;height:4.01781in"
alt="A blue circuit board with wires Description automatically generated" />

<img src="media/image14.png" style="width:3.90114in;height:5.20152in"
alt="Active Suspension Electronics Enclosure" />

## 6: Software 

The programming language used to program the Arduino microcontroller is
C/C++. There are two main functions inside the program, ‘void setup’,
during which the servo motors are set to the default position (90
degrees) and the gyroscope is calibrated and results of that calibration
are saved to EEPROM[^27] so the calibration is not required again even
after the system is powered off, and ‘void loop’, where the main program
runs.

The main program reads changes in the environment (changes in the lean
of the car) using the gyroscope in quaternions[^28] and converts them
into Euler angles[^29] due to enhanced ease of manipulation further in
the program.

Initially, the algorithm looked like that: the data is read from the
gyroscope, if the Euler angle is below 0 (with some uncertainty for more
stability), we first check if a servo is still within its limits, if it
is - we increase its value, if it is outside its limits, we decrease the
opposite servo to balance it out, if they are both out of limits - that
defines system’s limits as a whole. The same (but opposite) works for
values above 0. I decided to revise the program comprehensively,
shifting from a rigid structure with hardcoded values[^30] to a more
adaptive and responsive approach. This transition involves replacing
nested statements[^31] that presuppose limitations defined solely by the
developer (myself) with a system capable of dynamically adjusting to the
external environment.

Old version of the algorithm:

<img src="media/image15.png" style="width:3.50246in;height:4.96809in"
alt="A screen shot of a computer code Description automatically generated" />

The idea behind the new algorithm is Proportional-Integral-Derivative
control[^32] (The MathWorks, Inc., n.d.) which allows the suspension to
auto-regulate itself to the desired setpoint. The algorithm consists of
7 variables used to adjust the PID control, the variables were adjusted
using the trial-and-error method based on some tests, ensuring that the
suspension was quick enough to response to changes in the environment
while not to an extend where ‘wobbling’ of the whole car occurs where it
tries to adjust to little changes. Euler angles get multiplied by PID
variables and are either subtracted or added to the default position of
servo motors depending on the location of the servo motors within the
predefined constraints[^33].

New version of the
algorithm:<img src="media/image16.png" style="width:5.59574in;height:3.95393in"
alt="A screen shot of a computer program Description automatically generated" />

Finding the constraints of servo
motors:<img src="media/image17.jpeg" style="width:5.88298in;height:4.28946in"
alt="A person working on a machine Description automatically generated" />

## 7: Performance Assessment

To properly evaluate the effectiveness of the active suspension system
in rollover prevention a series of experiments and tests need to be
conducted.

First technique involved gradually changing the angle of the slope at
which the vehicle is resting until it the wheels on one side lift with
active suspension on and off. This would help to evaluate whether the
system will have any effect on the stationary tilt angle of the vehicle.
I have attached the back wheels of the car to two wood blocks to prevent
the car from rolling/sliding down. I have used an iPad with a spirit
level app installed to measure accurate angle to the horizon of
vehicle’s position. I have repeated the same experiment with vehicle
being parallel to the blocks (side tilting).

<img src="media/image18.png" style="width:6.26806in;height:3.52292in"
alt="A tablet on a table with a remote control Description automatically generated" />

The results were not as expected, the difference of the tilt angle was
small (around 2°) and the tilt angle was surpassingly high (around 80°
in pitch and around 52° in roll). The explanation to it might be that
the principal factor contributing to rollovers, which are linked to the
tilt angle of a vehicle, may be caused by abrupt environmental changes
rather than a gradual incline in the slope.

Therefore, I decided to do a proper field test by driving the vehicle
outside around different rocks and planks.

<img src="media/image19.png" style="width:6.26806in;height:4.70139in"
alt="A toy car on a wood deck Description automatically generated" />

I thought that reading tilt angles from a gyroscope and just adjusting
for those will prevent most of rollovers from happening, but the system
has proven to be a little unresponsive due to limitation in actuators’
reaction speed, therefore, under a certain condition the car with active
suspension on is winning to the car with the suspension off. As result,
other factors such as acceleration inputs need to be considered.

I concluded that the suspension does not work well at preventing all
types of rollovers, however, it effectively maintains contact between
all wheels and the ground, improving traction and reducing the
likelihood of certain types of rollovers. Specifically, it addresses
scenarios where rapid acceleration while one wheel is in the air can
shift the vehicle's weight onto that wheel and lead to its contact with
the ground. This abrupt grounding causes a sudden halt for that part of
the vehicle, but the rest of it continues to move due to its momentum.
Consequently, the vehicle may tilt towards the direction of the initial
fall, with the wheel that first touched down acting as a pivot. This can
lead to the opposite side lifting off the ground, increasing the risk of
a rollover. By ensuring that all wheels remain in contact with the
ground, the vehicle avoids those abrupt stops caused by wheel grounding
that can contribute to rollovers.

<img src="media/image20.png" style="width:4.43723in;height:2.49392in"
alt="A remote control car on a sidewalk Description automatically generated" />

Overall, the system is still quite raw and will require a lot of
improvements such as further PID control adjustments to prevent
wobbling, read steering and accelerator inputs from the RC transmitter
to counteract those before the vehicle starts tilting before it becomes
something valuable to have in a real-world use
case.<span id="_Toc161216122" class="anchor"></span>

# Evaluation

This project aimed at developing an active suspension system tailored
for off-road vehicles to mitigate the risk of rollovers, particularly on
uneven terrain. The project was driven by the need to enhance vehicle
stability in challenging environments, which is paramount for improving
safety measures in off-road navigation.

The project's inception involved a comprehensive review of existing
literature to identify gaps in current rollover prevention technologies.
This research highlighted the absence of active suspension systems in
the market that dynamically adjust a vehicle’s centre of mass to prevent
rollovers. Implementing the system on a 1/10th scale model car allowed
for a pragmatic approach to testing and development, facilitating the
exploration of innovative solutions within a manageable and
cost-effective framework.

Despite the project's ambition and the successful integration of various
engineering disciplines, including mechanical engineering and software
development, the performance evaluation revealed limitations. While the
system effectively maintained all wheels' contact with the ground,
enhancing traction, it was not universally effective against all types
of rollovers. This outcome suggests the necessity for further
refinement, potentially through the integration of advanced sensors and
more responsive control algorithms, to better anticipate and counteract
the dynamics leading to a rollover.

Reflecting on this project, it has been a great learning experience for
me, deepening my expertise in engineering and problem-solving.
Designing, implementing, and testing the active suspension system not
only sharpened my technical skills but also nurtured my ability to
approach challenges with resilience and innovation. This work enhanced
my knowledge in mechanical design, electronics, and software
development, illustrating the power of an interdisciplinary approach.
More than just advancing vehicle safety, this project has been
instrumental in my growth as an engineer, preparing me for future
challenges in the field.

# Bibliography

Wikipedia Foundation, 2022. *Tilt test (vehicle safety test).*
\[Online\]  
Available at:
<u>https://en.wikipedia.org/wiki/Tilt_test\_(vehicle_safety_test)</u>

Wikipedia Foundation, 2020. *Active rollover protection.* \[Online\]  
Available at:
<u>https://en.wikipedia.org/wiki/Active_rollover_protection</u>

Wikipedia Foundation, 2023. *Gyroscope.* \[Online\]  
Available at: <u>https://en.wikipedia.org/wiki/Gyroscope</u>

Wikipedia Foundation, 2023. *Crosswind stabilization.* \[Online\]  
Available at:
<u>https://en.wikipedia.org/wiki/Crosswind_stabilization</u>

Westhuizen, S. F. v. d. & Els, P. S., 2013. *Slow active suspension
control for rollover prevention.* \[Online\]  
Available at:
<u>https://www.sciencedirect.com/science/article/abs/pii/S0022489812000523</u>

Wikipedia Foundation, 2023. *Anti-roll bar.* \[Online\]  
Available at: <u>https://en.wikipedia.org/wiki/Anti-roll_bar</u>

Wikipedia Foundation, 2023. *Active suspension.* \[Online\]  
Available at: <u>https://en.wikipedia.org/wiki/Active_suspension</u>

Soliman, A. & Kaldas, M., 2019. Semi-active suspension systems from
research to mass-market – A review. *Journal of Low Frequency Noise,
Vibration and Active Control,* pp. 1-19.

Savaresi, S. M. et al., 2010. CHAPTER 2 - Semi-Active Suspension
Technologies and Models. *ScienceDirect,* pp. 15-39.

Li, I.-H. & Lee, L.-W., 2019. Design and Development of an Active
Suspension System Using Pneumatic-Muscle Actuator and Intelligent
Control. *Applied Industrial Technologies,* pp. 1-22.

Pranav Teli, V. T. S. Z. A. S., 2019. Study of Electromagnetic Damper.
*IJERT,* pp. 1-4.

Suspension Spot, 2018. *The Incredible Bose Active Suspension System.*
\[Online\]  
Available at:
<u>https://suspensionspot.com/blogs/news/the-incredible-bose-active-suspension-system</u>

Deutermann, W., 2002. *Characteristics of Fatal Rollover Crashes,*
Springfield: National Center for Statistics and Analysis Research and
Development.

Qazizadeh, A., 2017. *On Active Suspension in Rail Vehicles,* Stockholm:
KTH Engineering Sciences.

Wikipedia Foundation, 2024. *Vehicle rollover.* \[Online\]  
Available at: <u>https://en.wikipedia.org/wiki/Vehicle_rollover</u>

LastMinuteEngineers.com, n.d. *Interface MPU6050 Accelerometer and
Gyroscope Sensor with Arduino.* \[Online\]  
Available at:
<u>https://lastminuteengineers.com/mpu6050-accel-gyro-arduino-tutorial/</u>

The MathWorks, Inc., n.d. *What Is PID Control?.* \[Online\]  
Available at:
<u>https://www.mathworks.com/discovery/pid-control.html</u>

Howard, B., 2017. *Bose Sells Off Its Revolutionary Electromagnetic
Suspension.* \[Online\]  
Available at:
<u>https://www.extremetech.com/cars/259042-bose-sells-off-revolutionary-electromagnetic-suspension</u>

Valor Offroad LLC, n.d. *UNSPRUNG VS SPRUNG WEIGHT.* \[Online\]  
Available at:
<u>https://www.valoroffroad.com/blogs/the-source/how-wheel-weight-impacts-your-vehicle</u>

NIO, 2024. *NIO ET9 SkyRide Fully Active Suspension system test.*
\[Online\]  
Available at: <u>https://www.youtube.com/watch?v=WEr_sNp7XLM</u>

Acorn Industrial Services Limited, 2022. *How Does a Linear Actuator
Work?.* \[Online\]  
Available at:
<u>https://www.acorn-ind.co.uk/insight/how-does-a-linear-actuator-work/</u>

Cockfield, B., 2021. *Hackaday.* \[Online\]  
Available at:
<u>https://hackaday.com/2021/04/14/remote-controlled-car-gets-active-suspension/</u>

# Appendices

- All the source code written in C/C++ can be found in the following
  repository: <https://github.com/Kirillr-Sibirski/active-suspension>

- Recording of the front active suspension with manual control can be
  found here:
  <https://drive.google.com/file/d/16r3aaTe9VnJRDXSLNbqiRYK6pXKvDrG7/view?usp=sharing>

- Manual suspension height control on an improvised slope:
  <https://drive.google.com/file/d/1aXUrs_qTR0xxHQ3LDvsdVV4xdex2i2ZF/view?usp=sharing>

- Evaluating the system’s response to the change in tilt angle:
  <https://drive.google.com/file/d/18YsJDTEvEd6NJ3I7WRD_4ZL7b0EVNZqq/view?usp=sharing>

- Lab test of the system:
  <https://drive.google.com/file/d/1rtssHeAuVpRo8k0TcI1NMlGuokQumr_D/view?usp=sharing>

- Field test of the vehicle \#1:
  <https://drive.google.com/file/d/1TPocOsuFJqaSmIil11Vonk33jqSuq7Qu/view?usp=sharing>

- Field test of the vehicle \#2:
  <https://drive.google.com/file/d/1WIKq6focy2tiCaoCsJcmZQIBLOANLCEO/view?usp=sharing>

[^1]: A rollover happens when a vehicle flips over onto its side or roof
    from its normal position.

[^2]: Untripped rollovers happen when cornering forces destabilize the
    vehicle. As the vehicle rounds a corner, tire forces push it towards
    the curve's centre at ground level, while inertial effects push it
    away from the turn's centre, both acting through the centre of mass.
    These opposing forces cause the vehicle to roll outward. If the
    combined tire and inertial forces exceed the force of gravity
    through the centre of mass, the vehicle begins to overturn.

[^3]: Is the point at which the mass of a system is concentrated without
    affecting its behaviour under external linear forces.

[^4]: The car used is a highly modified Remo Hobby Crawler Series.

[^5]: Body roll is the sideways leaning movement of a vehicle while
    turning.

[^6]: Device designed to dampen and absorb the shock impulses generated
    by a moving vehicle's suspension system. Its primary function is to
    enhance the overall ride comfort and stability of the vehicle by
    controlling the oscillations and vibrations produced during motion.

[^7]: Throughout the essay, terms ‘damper’, ‘shock absorber’,
    ‘absorber’, ‘shock’ and ‘strut’ are used interchangeably,
    acknowledging that they may have specific technical distinctions in
    certain contexts.

[^8]: Combine a shock and spring assembly in one unit that is part of
    the structural make-up of the vehicle.

[^9]: The sprung mass (weight) refers to the parts that the suspension
    holds up, like the body and chassis in the middle of the car.

[^10]: The unsprung mass (weight) includes parts not supported by the
    suspension, such as the dampers, controls arms, and wheels.

[^11]: This is the angle at which a car can drive diagonally across a
    slope without tipping over.

[^12]: A linear actuator is a mechanical device that converts rotary
    motion in a circular direction into linear motion along a straight
    line (Acorn Industrial Services Limited, 2022).

[^13]: The body shell is the outer covering of the RC car, typically
    made of lightweight and durable materials such as polycarbonate. It
    serves both aesthetic and protective purposes.

[^14]: A servo motor consists of a motor, feedback control system, and a
    gearbox. It works by receiving electrical signals that instruct it
    to move to a particular position. The feedback system continually
    checks the motor's current position, adjusting its rotation until it
    matches the desired position. This precise control allows the motor
    to achieve accurate and controlled movement.

[^15]: A servo horn is a component attached to the output shaft of a
    servo motor, transmitting rotational motion to other mechanical
    parts or devices.

[^16]: The ability of a structure or component to withstand its intended
    load without experiencing failure, deformation, or collapse.

[^17]: Layer-by-layer additive process used to make a three-dimensional
    object using plastic. Creality Ender V3 SE printer was used.

[^18]: A thermoplastic monomer derived from renewable, organic sources
    such as corn starch or sugar cane. Was chosen due to its relatively
    low cost and simplicity to print.

[^19]: Physical programmable circuit board (accessible price. Arduino
    Nano is type of a board which is more compact in size and has a bit
    less computing than a traditional a microcontroller), used by
    hobbyists all around the world due to its simplicity and Arduino
    Uno.

[^20]: Using a 20kg/cm generic servo motor bought from China.

[^21]: Rechargeable battery of lithium-ion technology that uses a
    polymer electrolyte instead of a liquid one. Each cell produces
    3.7V, therefore, by connecting 2 of them in series, 7.4V output is
    achieved.

[^22]: A construction base used to build semi-permanent prototypes of
    electronic circuits without soldering.

[^23]: Electrical wire or cable, which is used to interconnect the
    components of a breadboard or other prototype or test circuit,
    without soldering.

[^24]: Communication protocol used for interfacing peripheral circuits
    to prototyping systems.

[^25]: Radiolink R6FG 2.4GHz

[^26]: Material for prototyping circuit boards. It is a thin, rigid
    sheet with holes pre-drilled at standard intervals across a grid.

[^27]: Type of non-volatile memory that allows the storage of data even
    when the Arduino is powered off or restarted.

[^28]: Mathematical concept used to represent rotations in
    three-dimensional space. Consists of w (scalar), x, y, z components.

[^29]: A set of three angles that represent the orientation of an object
    in three-dimensional space. Consists of roll, pitch and yaw.

[^30]: Hardcoded values refer to the practice of embedding specific,
    fixed numerical or textual data directly into the source code of a
    program as opposed to dynamic and flexible by introducing variables
    or configuration settings that can be easily adjusted without
    modifying the source code.

[^31]: Nested statements refer to a set of instructions or conditions
    within another set. It is like giving someone a list of tasks to do,
    and within each task, there could be additional instructions or
    conditions.

[^32]: Control loop mechanism used in control engineering.

[^33]: From 40 to 140 degrees. After these values, the system no longer
    raises or lowers the suspension due to circular motion of the servo
    motor but does the complete opposite.
