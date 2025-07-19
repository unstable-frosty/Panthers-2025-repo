## Table of contents

- [Introduction](#Introduction)
- [Hardware documentation](#hardware-documentation)
- <a href="https://github.com/unstable-frosty/Panthers-2025-repo/tree/8454e8c2bf0ceed4cd277130ab7a8011b173d936/src">Coding Documentation</a>
- [teamwork and ethic](#teamwork-and-ethic) 

---

# Introduction
Hello everyone! 
Welcome to our GitHub repository. In this repo, we will be documenting our journey in building our obstacle avoiding car, while also sharing some details about our software, design struggles and how we managed to overcome these challenges. We hope to inspire everyone who reads this documentation and spark an idea for their future solutions. Thank you for your visit, enjoy the ride!

# Team members
The team consists of two people and a "coach" or as we like to call her at home ✨mum✨.

Kareem Siblini  (Kareemsiblini@gmail.com)
Adam Siblini  (adamsiblini@gmail.com)
Ebtisam nassif (enassif@gmail.com)

# Experiences passed from different competitions

We are completely self taught developers and this is our first year competing in the future engineering category and our second year competing in the WRO. We have competed in the robomission category, this allowed our minds to prosper with creative designs which highly assist in the mechanical aspect of our robot. Not only did we gain an eye for design but, competing in line following competitions and breaking a 16 year record in one of them using our lego EV3 and maxing out its capabilities using a complex mathematical PD equation. However, still with all these skills, there were still many challenges to be waiting on us, from camera positioning to calibration and quadratic curve equations we have never used before.

# Designing our first robot prototype

For our first design we decided to take things slow and figure out a better and more efficient design. We initially used ackerman steering and a differential with the medium motors stacked on top of each other horizontally and we geared down the steering mechanism using bevel gears. We also used spike wheels for the drive base and the steering. All of this came with a disadvantage; the car was way too big and the steering angle was bad due to the size of the wheels.

![image alt](https://github.com/unstable-frosty/Panthers-2025-repo/blob/cc2bc7a8f7964c5d921e8d20fd3a8d1da6067953/v-photos/Old%20prototype/proto%20v1%20side.JPG)

# prototype v2 vs v1 

For our second prototype we decided to implement a major change, an upgrade that would flip our car's performance. We decided to utilize a bit of the car's height and re-adjust our motors vertically. We then removed the down gearing of the steering, using parallel steering instead of ackerman's with smaller wheels, which allowed the angle to turn up to a 100 degrees, without limiting the steering, connecting the motor directly to the steering axle with no gears. This came with alot of problems. The steering would either overshoot or undershoot and vibrate way too much. We were in a dilemma for 3 days until we decided to gear down the motor again, turns out the backlash from the gears would actually make the algorithm way more stable, an imperfection that was just right for our solution.

![image alt](https://github.com/unstable-frosty/Panthers-2025-repo/blob/f1d86ef3bd66ec246dbd942dfeb18111a9b46b35/v-photos/Old%20prototype/comparison%20proto%20v1%20vs%20v2.jpeg)

# final design V3(isometric)
![image alt](https://github.com/unstable-frosty/Panthers-2025-repo/blob/0c87d287e1318654dd969f27b1b47dae0c5ba3b7/v-photos/isometric%20photo.jpeg)

# teamwork and ethic

Since Adam and I are twin brothers, and we live under the same roof, we finalized the robot within a 3 week period with rigorous research on algorithms, and didnt have an issue with meetups, we each took a major part in the project as well as assisted a bitn in each others parts. For example, we split the work into 3 main parts; coding, designing, and documenting. Adam was responsible for the coding while I (kareem) was maintaining the documentation and the effects of the code on the design, specifically the camera placement and tuning. If one of us faced an issue with their work, we made sure to communicate and break down the issue into pieces to resolve it. We would usually work around 8-10 hours per day on average to make sure perfection of the design and coding while going through many iterations to fine tune it as much as possible according to our capabilities.

# Conclusion

To sum this all up, we have put a great amount of work and dedication into this project, and despite the challenges that we have faced throughout, we believe that it was very helpful and insightful, as we learned a lot about new algorithms and mechanisms. We would like to thank our Coach (AKA mum). We would like to apologize to our coach for conquering the house with the map and all the Lego pieces :).



