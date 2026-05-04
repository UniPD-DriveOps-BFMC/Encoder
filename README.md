<h2>
⚠️ Manufacturer recommends no more than three cycles of mounting and removal of the AMT top cover base. Multiple cycles can cause base fatigue over time and make the base break, affecting encoder performance. It is advised to remove the encoder together with the mounting bracket as a whole from the car.
</h2>
<h3>
ALGORITHM
</h3>
  
1. Read encoder count difference from timer
2. Add this difference to total encoder count
3. Apply drift correction using Z index: **Z CORRECTION**
   - Compute position error relative to one full revolution
   - Apply a small correction to the encoder offset
   - Update corrected encoder counts
4. Convert corrected counts to distance
5. Calculate speed using change in distance over time
6. If speed is very small, set it to zero
7. Apply low-pass filter to smooth the speed: **VELOCITY_LPF**
8. If filtered speed is very small, set it to zero
9. Integrate filtered speed over time to get filtered distance
10. Differentiate filtered speed over time to get acceleration
11. Store current values to use in the next iteration

Change **Z_CORRECTION_GAIN** and **VELOCITY_LPF_ALPHA** as needed!
