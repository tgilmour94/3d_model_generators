# 3D Model Generators

Parametric scripts for generating 3D-printable models.

## FinelineRack

[`FinelineRack/FinelineRack.py`](FinelineRack/FinelineRack.py) is a Fusion 360 script that builds a two-piece rack for Fineline applicator bottles: a cupped base with corner posts, and a top ring piece with flared lead-ins that press-fits onto the posts.

**Usage:** In Fusion 360, go to Utilities → Scripts and Add-Ins, create a new Python script, and paste in the file contents (or add this folder). Edit the parameters at the top of the file (rows, columns, hole diameter, heights, fit clearances; all in inches), then run it in a new design.
