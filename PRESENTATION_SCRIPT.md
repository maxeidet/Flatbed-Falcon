# Narration script for the final video

A proposal for what to say while `videos/flatbed_falcon_final.mp4` (3:06) plays. It's written for a calm speaking pace, about 140 words per minute, and each block takes at most about 90% of its part, so there's room to breathe. If you want to say more, use the Q&A section at the end, or pause the video. The video has its own captions, so you don't have to read them aloud. The narration explains *why*, and the captions show *what*.

- **Times** are video time (mm:ss). **Cue** marks a moment on screen to time a sentence against.
- The numbers come from the runs and are explained in [CONTROLLERS.md](CONTROLLERS.md). The last section lists the questions you're most likely to get, with the details that didn't fit in the narration.

---

## 0:00–0:11 · Title and "1 · A landing"

> Flatbed Falcon: a drone that lands by itself on a moving truck. We compare LQR with LQI, and a Kalman filter on and off.

## 0:11–0:35 · An LQI landing with the gate lamps

> This is LQI with the Kalman filter. Top left is the descent gate: position, speed and height must be within limits for one and a half seconds.

**Cue 0:15–0:18:** "asked to land", then LANDING_OK.

> Then it asks the truck, which says yes on a straight.

**Cue 0:23:** DESCEND.

> It descends…

**Cue 0:31:** touchdown.

> …and lands nine centimetres from the centre.

## 0:35–0:41 · "2 · LQR vs LQI"

> LQR versus LQI. Same method, but LQI also integrates the error.

## 0:41–1:10 · LQI and LQR side by side

> Same truck, same radio, same filter. LQI on the left, LQR on the right. Watch LQR's position lamp: it stays red.

**Cue 0:49–0:52:** LQI asks to land and gets permission.

> LQI gets into the gate and gets permission. LQR hangs almost a metre behind the bed, the whole time.

**Cue 0:57:** LQI descends.

> In the plot below, the orange line is flat at about point nine. That's a steady error, not noise.

## 1:10–1:36 · The phase portrait (2× speed)

> Why? Sideways is the error along the road, up and down is how fast it changes.

> Rotor drag pushes the drone back. Without an integral term, LQR can only hold that with a constant error.

**Cue around 1:17 and 1:23:** the truck changes speed.

> The jumps are the truck braking and speeding up. LQI's integral pulls it back into the box.

## 1:36–1:43 · Along-track still

> On average, LQR is 0.84 metres behind. LQI: one centimetre.

## 1:43–1:49 · "3 · Kalman filter on vs off"

> Now the same LQI, with and without the Kalman filter.

## 1:49–2:17 · "What the drone knows"

> The orange dots are the radio measurements. They trail behind the bed, because they're a tenth of a second old.

> The blue cross is where the drone aims. On the left the filter predicts forward and smooths: the cross is still. On the right it jitters.

**Cue: the bed-height plot (middle).**

> And look at the height: without the filter it jumps by ten centimetres.

## 2:17–2:37 · With and without Kalman side by side

> Sideways both are fine. But on the right the height lamp stays red: the drone hovers almost a metre too low.

**Cue 2:18:** the left one asks to land.

> So it never asks to land. The left one lands.

## 2:37–2:51 · Height and error chain stills

> Four metres with the filter, 3.17 without.

> The radio error is mostly latency, about 0.6 metres. After the filter: two centimetres.

## 2:51–3:06 · Results and end card

> LQR lacks integral action. Without the filter, the height is too noisy. LQI with the Kalman filter lands, nine centimetres from the centre. Thank you.

---

## Shorter version (if you want to say less)

If you'd rather let the video speak for itself, these six sentences are enough, one per section:

1. **0:00** "A drone that lands by itself on a moving truck, in simulation."
2. **0:11** "Top left is the descent gate: when position, speed and height are within limits, the drone asks the truck for permission and lands."
3. **0:41** "LQR, on the right, has no integral term, so rotor drag holds it about a metre behind the bed, and it never gets into the gate."
4. **1:49** "The radio delivers the position late and noisy; the Kalman filter predicts it forward to now and smooths it."
5. **2:17** "Without the filter the bed height is so noisy that the drone hovers 0.8 metres too low and never asks to land."
6. **2:51** "LQI with the Kalman filter is the combination that lands: nine centimetres from the centre."

## Likely questions, with short answers

- **How does the landing handshake work?** Once the gate has held for 1.5 s, the drone sends READY_TO_LAND. The truck answers LANDING_OK on a straight with at least 10 s left and holds a steady 4 m/s; then the gate has to hold again before the descent (0.5 m/s).
- **How big is the latency error?** 0.1 s × 6–8 m/s ≈ 0.6–0.8 m. The filter cancels it by predicting each measurement from the time it was taken to "now".
- **Why not PID?** We tested it. It lands too (0.04 m), but tracks worse than LQI (0.66 m against 0.38 m median). We left it out to keep the comparison clean: LQR and LQI come from the same method, and the only difference is the integral term.
- **Why did the drone without the filter end up *too low* rather than just shaking?** The noise reaches PX4's height control as noisy velocity commands. That it turns into a constant offset downward is something we've measured but not fully explained. Our guess is an asymmetry in PX4's vertical control.
- **Isn't the gate too strict?** The height window is stricter than safety requires: descending from 3.2 m would have been fine. But the gate caught that the drone wasn't flying at its commanded height, which is a sign something is wrong. A better version would compare against the drone's own setpoint. See CONTROLLERS.md, section 2.4.
- **One run per controller?** Yes. The landing times and touchdown errors are single runs and vary. The steady LQR lag and the tracking errors are averages over minutes of tracking, and they are reliable.
- **0.09 m in the video but 0.02 m in the controller's log?** The video measures against the *true* bed, while the controller measures against its Kalman estimate. Both are well inside the bed.
- **Why was the no-filter run better sideways (0.20 m)?** It isn't comparable: that truck never slowed down for a landing, so it drove a different pattern. One run against one run.
