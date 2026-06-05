# AS5048A STM32 Sensor Project

STM32CubeIDE project for the AS5048A magnetic sensor and motor control application.

## Project structure

- `Inc/` - project headers and configuration files
- `Src/` - application and motor control source files
- `Drivers/` - STM32 HAL and CMSIS drivers
- `STM32CubeIDE/` - IDE workspace files

## Publish to GitHub

1. Make sure the remote URL is set cleanly:
   ```bash
git remote set-url origin https://github.com/Gustar00/AS5048A_STM32.git
```
2. Add files:
   ```bash
git add .gitignore README.md
```
3. Commit:
   ```bash
git commit -m "Add .gitignore and README"
```
4. Push to GitHub:
   ```bash
git push -u origin main
```

## Notes

- Avoid committing build artifacts from `Debug/` or `STM32CubeIDE/Debug/`.
- If Git prompts for authentication, use your GitHub credentials or a PAT configured through the OS credential manager.
