$ErrorActionPreference = "Stop"

$glfwUrl = "https://github.com/glfw/glfw/releases/download/3.4/glfw-3.4.bin.WIN64.zip"
$zipPath = "glfw.zip"
$extractPath = "glfw_temp"
$depsPath = "deps"

Write-Host "Creating dependencies directory..."
New-Item -ItemType Directory -Force -Path $depsPath | Out-Null

Write-Host "Downloading GLFW from $glfwUrl..."
Invoke-WebRequest -Uri $glfwUrl -OutFile $zipPath

Write-Host "Extracting GLFW..."
Expand-Archive -Path $zipPath -DestinationPath $extractPath -Force

# Find the extracted folder (it usually has a version number)
$extractedRoot = Get-ChildItem -Path $extractPath -Directory | Select-Object -First 1
$rootPath = $extractedRoot.FullName

Write-Host "Configuring include files..."
Copy-Item -Path "$rootPath\include" -Destination "$depsPath" -Recurse -Force

Write-Host "Configuring library files (MinGW-w64)..."
New-Item -ItemType Directory -Force -Path "$depsPath\lib" | Out-Null
Copy-Item -Path "$rootPath\lib-mingw-w64\*" -Destination "$depsPath\lib" -Force

Write-Host "Cleaning up..."
Remove-Item -Path $zipPath -Force
Remove-Item -Path $extractPath -Recurse -Force

Write-Host "Success! GLFW installed to ./deps"
Write-Host "You can now compile with: -I./deps/include -L./deps/lib -lglfw3 -lopengl32 -lglu32 -lgdi32"
