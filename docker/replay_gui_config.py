"""Copies Gazebo's playback GUI config and adds (or adjusts) a video recorder that records
on sim time, so the video plays at real speed. Lockstep is off: it froze the window
whenever the replay was paused or at its end.
Used by replay.sh inside the sim container:  python3 replay_gui_config.py <playback config> <out>
"""
import re
import sys

RECORD = ('<record_video><use_sim_time>true</use_sim_time><lockstep>false</lockstep>'
          '<bitrate>8000000</bitrate></record_video>')

src, dst = sys.argv[1], sys.argv[2]
text = open(src).read()
block = re.search(r'<plugin filename="VideoRecorder".*?</plugin>', text, flags=re.S)
if block:
    body = re.sub(r'\s*<record_video>.*?</record_video>', '', block.group(0), flags=re.S)
    body = body.replace('</plugin>', RECORD + '</plugin>')
    text = text[:block.start()] + body + text[block.end():]
else:
    text += f'\n<plugin filename="VideoRecorder" name="VideoRecorder">{RECORD}</plugin>\n'
open(dst, 'w').write(text)

# replay.sh --auto films by asking this plugin for a screenshot of every frame.
if not re.search(r'<plugin filename="Screenshot"', text):
    text += '\n<plugin filename="Screenshot" name="Screenshot"></plugin>\n'
    open(dst, 'w').write(text)
