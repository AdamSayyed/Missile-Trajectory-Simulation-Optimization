$ErrorActionPreference = "Stop"

if (-not (Test-Path "deps/include/GLFW/glfw3.h")) {
    Write-Warning "Dependencies not found. Running setup_dependencies.ps1..."
    .\setup_dependencies.ps1
}

Write-Host "Compiling Missile Simulation..."
g++ -o missile_sim_3d.exe main.cpp -I./deps/include -L./deps/lib -lglfw3 -lopengl32 -lglu32 -lgdi32

if ($LASTEXITCODE -eq 0) {
    Write-Host "Build successful! Run with: .\missile_sim_3d.exe" -ForegroundColor Green
} else {
    Write-Host "Build failed." -ForegroundColor Red
}
