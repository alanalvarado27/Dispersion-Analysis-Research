param(
    [string]$GlfwRoot = "C:\Users\alana\projects\libraries\glfw-3.5.1",
    [string]$GladInclude = "C:\Users\alana\projects\libraries",
    [string]$GladSource = "C:\Users\alana\projects\heat_transfer_calcs\glad.c",
    [string]$GlfwLib = '',
    [string]$SolidWorksDir = 'C:\Program Files\SOLIDWORKS Corp\SOLIDWORKS (2)'
)
$ErrorActionPreference = 'Stop'
Set-Location $PSScriptRoot
if (-not (Get-Command cl.exe -ErrorAction SilentlyContinue)) {
    throw 'Run this in an x64 Native Tools/Developer PowerShell for Visual Studio (cl.exe must be available).'
}
$GlfwRoot = (Resolve-Path $GlfwRoot).Path
$GladInclude = (Resolve-Path $GladInclude).Path
$GladSource = (Resolve-Path $GladSource).Path
if (-not $GlfwLib) {
    foreach ($folder in @('lib-vc2022','lib-vc2019','lib-vc2017','build\src\Release','build\src','src\Release','src')) {
        $candidate = Join-Path $GlfwRoot "$folder\glfw3.lib"
        if (Test-Path $candidate) { $GlfwLib = $candidate; break }
    }
}
if (-not $GlfwLib -or -not (Test-Path $GlfwLib)) {
    throw 'Pass -GlfwLib with the full path to your x64 static glfw3.lib (not glfw3dll.lib).'
}
$GlfwLib = (Resolve-Path $GlfwLib).Path
foreach ($item in @("$GlfwRoot\include\GLFW\glfw3.h", "$GladInclude\glad\glad.h",
    "$GladInclude\KHR\khrplatform.h", $GladSource, "$SolidWorksDir\sldworks.tlb", "$SolidWorksDir\swconst.tlb")) {
    if (-not (Test-Path $item)) { throw "Missing dependency: $item" }
}
New-Item -ItemType Directory -Force vendor,build | Out-Null
# Official upstream headers. Existing files are reused; internet is needed only on first build.
$headers = @(
    @('json.hpp','https://raw.githubusercontent.com/nlohmann/json/v3.12.0/single_include/nlohmann/json.hpp'),
    @('stb_image.h','https://raw.githubusercontent.com/nothings/stb/master/stb_image.h'),
    @('stb_image_write.h','https://raw.githubusercontent.com/nothings/stb/master/stb_image_write.h')
)
foreach ($entry in $headers) {
    $target = Join-Path 'vendor' $entry[0]
    if (-not (Test-Path $target)) {
        Write-Host "Downloading $($entry[0]) from its official repository..."
        Invoke-WebRequest -UseBasicParsing -Uri $entry[1] -OutFile "$target.tmp"
        Move-Item "$target.tmp" $target
    }
}
$swPath = $SolidWorksDir.Replace('\','/')
@"
#pragma once
#define SWORKS_TLB "$swPath/sldworks.tlb"
#define SWCONST_TLB "$swPath/swconst.tlb"
"@ | Set-Content -Encoding ascii solidworks_paths.hpp
& cl.exe /nologo /MD /O2 /c /TC "/I$GladInclude" $GladSource /Fobuild\glad.obj
if ($LASTEXITCODE -ne 0) { throw 'GLAD compilation failed.' }
& cl.exe /nologo /MD /EHsc /std:c++17 /O2 /W3 /utf-8 "/I$GlfwRoot\include" "/I$GladInclude" /Ivendor `
    solidworks_image_calibration.cpp build\glad.obj /Fobuild\ /Febuild\sketch_to_solidworks.exe `
    /link $GlfwLib opengl32.lib user32.lib gdi32.lib shell32.lib
if ($LASTEXITCODE -ne 0) { throw 'C++ compilation/link failed. See the first compiler error above.' }
Write-Host 'Built build\sketch_to_solidworks.exe'
