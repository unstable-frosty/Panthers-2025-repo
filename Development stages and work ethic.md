# 📘 **Development Stages (Engineering Overview)**  

---

## 📝 **1. Planning Phase**
Before building, we established a clear engineering direction to avoid wasted time and redesign loops.

**During this phase, we:**
- Defined the requirements of the WRO Future Engineers category and identified the technical functions our robot needed (accurate look-ahead sensing, smooth steering, stable behavior).
- Explored multiple steering mechanisms—**Ackermann and parallel**—and evaluated their geometric constraints and complexity.
- Sketched chassis concepts prioritizing **compactness, sensor visibility, and wiring accessibility**.
- Mapped out component placement, wheel sizing, motor orientation, and overall spatial constraints.
- Divided roles based on strengths:  
  **Kareem** on design, documentation, and sensory alignment;  
  **Adam** on coding, logic development, and control behavior.
- Built an iterative workflow centered on **prototype → test → document → optimize**.

> This planning stage created a solid foundation for a structured, engineering-focused development cycle.

---

## 🧭 **Background & Previous Experience**
Before entering the Future Engineers category, we built our foundation through years of self-driven learning and previous WRO participation. We are fully self-taught developers, and although this is our **first year** in Future Engineers, it is our **second year** competing in the WRO overall.

Our previous experience in the **RoboMission** category played a huge role in shaping our engineering mindset. The creative freedom of RoboMission forced us to approach problems with open-ended design thinking, which later became extremely valuable when building our obstacle-avoiding robot. It helped us develop a strong mechanical intuition, especially when designing compact robots under strict constraints.

We also gained experience in **line-following competitions**, where we pushed a LEGO EV3 system to its limits—breaking a **16-year standing record** using a finely tuned mathematical **PD control equation**. These challenges taught us precision, control theory, and efficient algorithm design.

Despite all these skills, the Future Engineers category introduced new obstacles:  
- camera placement and stabilization,  
- field-dependent calibration,  
- and even working with new mathematical concepts like **quadratic curve equations**.

These experiences shaped not only our workflow but also our determination to continuously adapt, learn, and engineer better solutions.

---

## 💡 **2. Concept Development**
Once the plan was set, we refined our ideas into early mechanical models and layout options.

**Our goals during concept development:**
- Create a chassis with enough turning freedom for tight maneuvers.
- Ensure the ultrasonic had a **clear forward “look-ahead” line**, crucial for fast reaction distance.
- Balance the robot’s size with maneuverability and mechanical simplicity.
- Identify weaknesses early through sketches and mini assemblies before committing to a full build.
- Prepare for multiple prototyping rounds with clear fallback options.

> Concept development gave us clarity on what to expect from each iteration and narrowed down our design path.

---

# 🛠️ **3. Prototyping Phase**

---

## 🔧 **Prototype V1 — Initial Exploration**

### **Engineering Characteristics**
- Ackermann steering paired with a differential drivetrain.  
- Medium motors mounted horizontally and stacked.  
- Steering geared down with bevel gears to smooth movements.  
- Spike wheels used for both drive and steering.  
- Ultrasonic sensor positioned **mid-chassis** due to blocked front space.

### **Issues Identified**
- Robot footprint became **too large**, reducing turning radius and limiting steering angle.  
- Large front wheels blocked us from adding ultrasonic sensors in the front.  
- The sensor being in the middle under the chassis created **late reaction time** for .....
- In open challenge tests, the robot frequently overcorrected due to inaccurate distance readings.

### ✅ **Engineering Insight from V1**  
Placing the ultrasonic at the front was essential.  
Ultrasonic sensor position was extremely inefficient and was the root problem of the robot

### 📷 **Prototype V1 Image**  
![Prototype V1](https://github.com/unstable-frosty/Panthers-2025-repo/blob/cc2bc7a8f7964c5d921e8d20fd3a8d1da6067953/v-photos/Old%20prototype/proto%20v1%20side.JPG)


---

## 🔧 **Prototype V2 — Structural Redesign**

### **Major Engineering Changes**
- Motors reoriented **vertically**, reducing the width and freeing space.  
- Steering system replaced with simplified **parallel steering** geared from 8 tooth to 16 tooth bevel gear to increase encoder resolution..  
- Smaller wheels added in the front, allowing nearly **100° of steering rotation**.  
- First attempt at **direct steering drive** without gears.

### **Challenges Introduced**
- Direct steering became **too responsive**, causing overshoot, undershoot, and jittering.  
- Micro-movements translated instantly into large steering corrections.  
- Algorithm tuning became extremely sensitive and unstable.

### ✅ **Breakthrough Solution**
- Reintroducing **gear reduction** added a small amount of **mechanical backlash**, which unexpectedly **damped micro-corrections**.  
- This created **smoother, more predictable steering behavior** and simplified algorithm tuning.

### ✅ **Engineering Insight from V2**  
Controlled mechanical imperfections—like backlash—can **stabilize** control loops instead of harming them.

### 📷 **Prototype V1 vs V2 Comparison**  
![V1 vs V2](https://github.com/unstable-frosty/Panthers-2025-repo/blob/f1d86ef3bd66ec246dbd942dfeb18111a9b46b35/v-photos/Old%20prototype/comparison%20proto%20v1%20vs%20v2.jpeg)


---

# 📊 **4. Testing & Evaluation**

### **Testing Focus Areas**
- Sensor reaction time after enabling forward look-ahead.  
- Steering stability across different correction frequencies.  
- Influence of wheel sizes and angles on maneuverability.  
- Chassis balance and structural vibration during sharp turns.  
- Repeatability across multiple test runs.

### ✅ **What Testing Revealed**
- Front-mounted ultrasonic dramatically improved reaction timing.  
- Parallel steering with small amount backlash provided the smoothest, most consistent control.  
- Compact front geometry enabled tighter steering without mechanical strain.

---

# 🔧 **5. Optimization Phase**
After identifying successful configurations, we refined the design for robust performance.

**Key optimizations included:**
- Fine steering tuning around the backlash window for predictable corrections.  
- Improved sensor alignment (camera + ultrasonic) for consistent visibility.  
- Rebalancing weight to reduce drift and uneven turns.  
- Cleaning wiring paths to prevent obstruction and simplify debugging.  
- Simplifying subsystems for easier maintenance and quick adjustments.

---

# 🚀 **6. Application Phase**
This is where the robot moved from “working prototype” to “competition-ready system.”

**Application achievements:**
- Algorithms were adapted to work *with* mechanical backlash, not against it.  
- Sensor data and steering logic were combined to create stable, early reactions to obstacles.  
- Mechanical behavior and software tuning were synchronized for predictable, repeatable runs.  
- The final robot delivered consistent performance across different field setups and lighting conditions.

### 📷 **Final V3 Isometric View**  
![V3 Isometric](https://github.com/unstable-frosty/Panthers-2025-repo/blob/0c87d287e1318654dd969f27b1b47dae0c5ba3b7/v-photos/isometric%20photo.jpeg)


---
# ❗ **Issues faced in International practice day**
-We found out that there was a strong magnetic field around the maps, therefore we decided to disable the magnetometer inside our gyro and used only accelerometer and the gyro. This helped remove the effect of the electromagnetic field on our robot.

# 🤝 **Work Ethic & Team Collaboration**

## **Team Collaboration**
- Tasks were divided logically—coding (Adam), design & documentation (Kareem)—but both of us contributed to each other’s work whenever needed.  
- Working in the same environment allowed rapid testing, immediate feedback, and fast iteration cycles.  
- Difficult problems were solved together by combining hardware and software perspectives.

## **Problem-Solving Approach**
1. Identify the issue  
2. Break it into manageable components  
3. Test a targeted solution  
4. Evaluate results  
5. Iterate until stable  

## **Documentation Practice**
- Detailed notes after each test  
- Photos capturing build stages  
- Logs of mechanical and algorithmic changes  
- Failures documented as carefully as successes  

## **Core Values**
- **Dedication:** long work sessions with constant iteration  
- **Consistency:** daily improvements and disciplined routine  
- **Curiosity:** exploring unconventional fixes (like using backlash)  
- **Team Unity:** always supporting each other  
- **Reflection:** learning from every result to guide the next step  


--


