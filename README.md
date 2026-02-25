## ESP-IDF Project Setup

This project is safe to clone on another PC. Build artifacts and machine-local config are intentionally ignored in Git.

### Prerequisites

- ESP-IDF installed (matching project version)
- Correct target toolchain installed

### First-time setup after clone

1. Clone the repository.
2. Create your local secrets file from template:

	**PowerShell**
	```powershell
	Copy-Item .\main\secrets.example.h .\main\secrets.h
	```

3. Edit `main/secrets.h` with your real Wi-Fi and InfluxDB credentials.
4. Build the project:

	```bash
	idf.py build
	```

### Notes

- `main/secrets.h` is ignored by Git and must be created on each machine.
- `build/`, `sdkconfig`, and `sdkconfig.old` are generated locally and ignored.
- Project defaults are tracked in `sdkconfig.defaults` and target-specific defaults files.

