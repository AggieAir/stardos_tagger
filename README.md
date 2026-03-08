# STARDOS Tagger

dependencies:

ubuntu: 
    libexiv2-dev

python:
    py3exiv2

The STARDOS Tagger node is a crucial component in all of STARDOS' photogrametry
pipelines. This node consumes GPS data produced by the autopilot and adds it to
images as EXIF tags, creating a GeoTIFF.

Significant work has gone into reverse-engineering the format of EXIF tags that
are applied by professional cameras such as the cameras produced by Micasense,
leading to a highly-compatible format.

Make sure to fill in details specific to your camera, including make and model,
as well as physical attributes such as focal length, when configuring this node
in StarCommand.
