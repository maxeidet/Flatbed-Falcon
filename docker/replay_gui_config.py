"""Copies Gazebo's default GUI config and makes its video recorder record on sim time,
in lockstep with the replay, so every frame is captured however slow rendering is.
Used by replay.sh inside the sim container:  python3 replay_gui_config.py <default> <out>
"""
import re
import sys

RECORD = ('<record_video><use_sim_time>true</use_sim_time><lockstep>true</lockstep>'
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
