# Pill Counting Controller
#### A microcontroller program used to keep track of and remind user of pill intake

# Overview
![image](https://github.com/user-attachments/assets/94a347ec-b3d8-4ca3-80d5-3a4b1fe50010) \
\
Picture of the MSP430FR4133 microcontroller. There are three onboard push buttons (P1.2 and P2.6 are labelled) and an LCD screen. The LCD screen is used to display the user interface in which the user uses the onboard push buttons to navigate. The ground pin and one of the GPIO pins are used to connect to the buzzer for alarm functionality.
![image](https://github.com/user-attachments/assets/7c08a3ea-993a-49fc-a764-630dad027aee) \
\
Preliminary flowchart for general use of the device. This implements the software and hardware design outputs.
![image](https://github.com/user-attachments/assets/ad243e69-5eb1-4fa4-bd59-39927cb54d4c) \
\
High-level sketch of the device (mechanical sketch design output).

# Description
My team and I decided to solve the issue of pill-intake of patients for a third-year design course. THe focus was mainly towards the economics side of the project, taking into consideration the economic burden of missing regular medication intake. The prototype is more proof-of-concept rather a fully fleshed-out product. Regardless, a major aspect of the prototype was its function. The bare minimum requirements were an alarm function and an up-down counter.

Using the MSP430 microcontroller I designed a program which took a user's preferred alarm setting to periodically remind the patient to take their medication. The alarm triggered at the same time each day using the RTC of the board, and activated a buzzer with a manually-coded pulse sequence. This would remain active until the user pressed the pill count-down button on the board. When the pill counter reaches zero, and if a user presses the down button again, a message would appear letting the user know to restock on their supply. As well, the buzzer would remain active if the total number of pills the user had was below a certain threshold, which the default case is five.

A full report of the project, including the program section, is included in this repository titled "Team37_report".

# Technologies
I used the following to create this program:
- MSP430Fr133 microcontroller
- Generic buzzer
- Code Composer
- C

# Learnings
Besides the economic aspect of the project, this project strengthened my foundation in microntrollers and their utilities. I learned how to utilize the RTC of the board in order to periodically trigger an interrupt which would send a pulse alarm to an external component. I also learned how to trigger event-cases, such as when the quantity of medication falls below a threshold and when a user attempts to decrease the count below zero.

# Authorship
I developed the mojority of the code myself, taking feedback from a team member regarding the pill-down function to make it more efficient. You can check out more of my work in my portfolio:

#### [Visit my portfolio](https://portfolio-ompatel.netlify.app/)
