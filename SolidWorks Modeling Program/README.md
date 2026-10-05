# Offline image-to-SolidWorks: straight edges, labeled dimensions, shaded material

## Update your existing project

Copy these THREE files from this ZIP into your existing source directory:
- solidworks_image_calibration.cpp (replace the old compiled source)
- offline_geometry.hpp (replace)
- dimension_constraints.hpp (new, required)

Keep your existing build.ps1, json.hpp, stb_image.h, GLAD, GLFW,
solidworks_paths.hpp, and MSVC/ATL/SolidWorks installation. This is an update
package, not a standalone installation or a precompiled executable. The source
no longer needs your older geometry.hpp. Compile as C++17 or newer.

From your Developer Command Prompt, for your currently installed VS BuildTools:

```bat
call "C:\Program Files (x86)\Microsoft Visual Studio\18\BuildTools\VC\Auxiliary\Build\vcvarsall.bat" x64
cd /d "C:\Users\alana\projects\MEEN 491H\solidworks_llama_project\solidworks_llama"
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\build.ps1
```

If your installation path differs, use its x64 Native Tools Command Prompt.
Both GLFW and the compiler target must be x64. Rebuild after changing source or
headers, but not for a new image/JSON.

## Straightened spiral pattern

Replace pattern_full.json with this version. It has 16 closed contours and
333 points, down from 1035. Long intended straight runs are represented by
single lines; pixel noise is removed. The manila regions remain solid; dark
lines are gaps. The image crop is the outer extent. Small corner details are
approximate. pattern_trace_preview.png shows the filled material interpretation.

```bat
.\build\sketch_to_solidworks.exe ".\input_pattern.png" ".\pattern_full.json"
```

The straightened JSON itself also works with the preceding executable. The new
executable is required to enforce labeled dimensions and explicit material roles.
The old upper_center_spiral demo is omitted from this updated bundle because it
represented the black stroke as material, rather than your intended manila region.

## Shaded drawing with variables: runnable example

Copy dimension_demo.png and dimension_demo.json into the project directory:

```bat
.\build\sketch_to_solidworks.exe ".\dimension_demo.png" ".\dimension_demo.json"
```

This drawing shows a shaded square surrounding an unshaded circle. The program
asks for d (square side) and D (hole diameter), both in mm. Try d=80 and D=20.
The resulting part is a square plate with a circular through-hole. Keep D<d.
Arrows, labels, and hatch strokes are not part geometry.

For a new drawing, upload the image and CHATGPT_PROMPT.txt here. ChatGPT reads
intended edges, labels/arrows, and shading and produces the JSON. The C++ program
loads that JSON; it has no OCR/image segmentation model and makes no API calls.
The optional `material` description explains the chosen interpretation. The
actual contours and their solid/hole roles determine what gets extruded.

## New dimension workflow

1. If dimensions exist, the console asks for each unique case-sensitive name.
   A printed numeric value becomes an editable default (Enter accepts it).
   A symbolic value has no invented default. All inputs are positive mm.
2. The first dimension establishes an initial scale, so two-point calibration
   is skipped. If no dimensions exist, the original two-click calibration runs.
3. Review the trace over the original image. Drag points and press Enter.
4. The solver applies the supplied dimensions and geometric relationships.
   Review the resized sketch in a second, geometry-only window. Its proportions
   may differ from the original drawing, as requested by the numeric values.
   The view fits the result even when resized geometry extends outside the image.
5. Dragging in the second window reapplies constraints on mouse release. A drag
   that cannot satisfy the constraints is undone. Other free coordinates can
   move. R restores the stage's starting geometry; Escape cancels.
6. Enter validates closure, crossings, and material roles, then asks for extrusion
   depth. The program saves reviewed_sketch.json and session.json before starting
   SolidWorks. `--preview-only` saves without starting SolidWorks.
7. Save the created part in SolidWorks using Ctrl+S.

Point insertion/deletion is disabled for sketches with dimensions/relations so
references cannot be silently broken. For plain sketches, right-click a line to
split it; Delete removes an eligible vertex between two lines.

reviewed_sketch.json preserves roles, dimensions, relations, values, and material
notes, so it can be loaded again. session.json records applied mm values, origin,
scale source, points in meters, and extrusion depth; it is not an import file.

Supported dimensions: point distance, horizontal/vertical separation, circle
radius/diameter. Supported relationships: horizontal, vertical, equal lengths,
parallel, perpendicular, and x/y midpoint. See CHATGPT_PROMPT.txt for exact JSON.
A rectangle needs horizontal/vertical relationships as well as its dimensions;
a square also needs equal adjacent lengths. Only relationships present in JSON
are enforced. For underconstrained geometry, the solver stays near the trace.

A square plate can have one d parameter; a rectangle can have independent width
and height parameters. Repeated labels use the same value. Labels are
case-sensitive. Inconsistent or impossible constraints fail before CAD creation.

## Limits and verification

The numerical solver applies dimensional coordinates before CAD creation. It does
not add native SolidWorks driving dimensions, equations, or fully constrained
sketch relations. Those are a separate future integration. Algebraic dimension
expressions, angle values, tangency constraints, splines, and multi-depth features
are not implemented. Circles and three-point arcs remain native CAD entities.

The first physical dimension is authoritative for size; pixel scale is only an
initial layout estimate. A normal image with no labels still needs a real distance
for calibration. Shading and annotation interpretation happen in ChatGPT and must
be encoded in the file; arbitrary replacement images alone are not analyzed.

Compiled and tested the actual parser/serializer, geometry checks, and dimension
solver in portable C++17. Tests cover independent rectangle width/height/hole
size, square-side equality, centered holes, single-d input, constrained dragging,
conflicting-dimension rollback, explicit solid/hole nesting validation, and
reviewed-file serialization/reload. Both the straightened pattern and delivered
dimension demo passed. The curve intersection screen remains sampled; SolidWorks
is the final extrusion check.

The Windows/OpenGL/COM application cannot be compiled or run in this Linux
workspace. The preceding version ran on your machine; the revised viewer and
terminal flow still require rebuilding and checking there.
