# Protofusion usermod

This usermod will

- Send analog and digital readings to an OSC server at a configurable IP address
- Start a custom ArtNet server that sets pixels ONLY on strips that are **frozen**
- Use analog reading to control number of pixels per segment
- Use analog reading to control brightnes / intensity of internal WLED effects (by segment)

## API

The LumaPXL node can send and receive OSC messages which are described below. Sent messages are transmitted to the IP address configured in the usermod.

### Send

- ```/[hostname]/digitalX``` where ```x`` is an integer from 0-2 for onboard GPIO, or 0-28 for expander GPIO
  - Sent only on change
  - Value is ```0``` or ```1```
  - Sent with ```1``` on press, ```0``` on release


- ```/[hostname]/analogX``` where ```x``` is 0 or 1
  - Value is float between 0.0 and 1.0
  - Sent on every loop iteration (rate set by usermod)
  - Sent on change of more than 0.01 if "only send on change" is set

### Receive

- ```/mod1/value``` with 1 float arg
  - Float 1: Value for strip modulation 1 (0.0-1.0)
  - Can be used instead of analog reading to modulate strip

- ```/mod2/value``` with 1 float arg
  - Float 1: Value for strip modulation 1 (0.0-1.0)
  - Can be used instead of analog reading to modulate strip

- ```/digital0/value``` with 1 integer arg
  - Integer 1: State (0=off 1=on)
  - Sets GPIO configured in usermod

- ```/strip/direction``` with 2 integer args
  - Integer 1: Segment ID starting at 0
  - Integer 2: Direction (0=normal 1=reverse)

- ```/strip/intensity``` with 2 integer args
  - Integer 1: Segment ID starting at 0
  - Integer 2: Intensity (0-255)

- ```/strip/freeze``` with 2 integer args
  - Integer 1: Segment ID starting at 0
  - Integer 2: Freeze (0=artnet enabled, 1=internal effects)

- ```/strip/opacity``` with 2 integer args
  - Integer 1: Segment ID starting at 0
  - Integer 2: Opacity (0-255)

- ```/strip/effect``` with 2 integer args
  - Integer 1: Segment ID starting at 0
  - Integer 2: Effect ID (see f/w for IDs)




