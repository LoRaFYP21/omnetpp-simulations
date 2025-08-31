# README

## Prerequisites

- OMNeT++ 5.3
- INET Framework 3.7.1
- Git
- MinGW (if not already installed, instructions provided below)

---

## Part 1: How to Set Up OMNeT++ 5.3

1. Download OMNeT++ 5.3 from the official GitHub releases:  
   [https://github.com/omnetpp/omnetpp/releases/tag/omnetpp-5.3](https://github.com/omnetpp/omnetpp/releases/tag/omnetpp-5.3)

2. Download the file: `omnetpp-5.3-src-windows.zip`.

3. Extract it to a folder that does **not** contain:
   - Long paths
   - Folder names with spaces  
   ✅ Correct: `D:\Folder_Sample`  
   ❌ Incorrect: `D:\Example Folder`

4. After extraction, you should have a directory like:  
   `D:\Folder_Sample\omnetpp-5.3`

5. Navigate to `D:\Folder_Sample\omnetpp-5.3` and open the `configure.user` file using a text editor (e.g., Notepad, VS Code).

6. Ensure the following setting is configured:
   ```bash
   PREFER_CLANG=no
   ```
   If it is set to `yes` by default, change it to `no`.

7. Run the `mingwenv.cmd` file inside the `omnetpp-5.3` directory. This will open a MinGW shell.

---

### 📌 Installing MinGW (If Not Already Installed)
If MinGW is not installed:
- Visit [https://sourceforge.net/projects/mingw](https://sourceforge.net/projects/mingw)
- Download and install MinGW
- During installation, make sure to select:
  - `mingw32-gcc-g++`
  - `mingw32-make`
- Add the `bin` folder of your MinGW installation (e.g., `C:\MinGW\bin`) to the system PATH.

---

8. In the MinGW shell, run the following command:
   ```bash
   ./configure
   ```
   This may take a few minutes.

9. Once completed, then type:
   ```bash
   make
   ```

10. Once completed, start OMNeT++ by typing:
   ```bash
   omnetpp
   ```
11. Proceed with the default settings in any pop-up windows.

---

## Part 2: How to Set Up INET Framework

1. Download INET 3.7.1 from:  
   [https://inet.omnetpp.org/Download.html](https://inet.omnetpp.org/Download.html)

2. Extract the zip file to the same `D:\Folder_Sample` directory.

3. You should now have two folders inside `Folder_Sample`:  
   - `omnetpp-5.3`  
   - `inet`

4. Open OMNeT++, then navigate to:  
   `File → Import → General → Existing Projects into Workspace`

5. In the dialog:
   - Click **Browse** and select the path: `D:\Folder_Sample\inet`
   - Click **Finish**

6. In the **Project Explorer** (usually on the left), you will now see the `inet` project.

7. Right-click on the `inet` project and select **Build Project**.  
   It will compile successfully.

---

## Part 3: Importing `omnetpp-mesh-tester` Project

1. In OMNeT++, go to:  
   `File → Import → Git → Projects from Git`

2. Enter the repository URL:  
   [https://github.com/LoRaFYP19/omnetpp-mesh-tester.git](https://github.com/LoRaFYP19/omnetpp-mesh-tester.git)

3. Complete the Git import steps. This will load the `omnetpp-mesh-tester` project into OMNeT++.

4. Once imported, right-click on the `omnetpp-mesh-tester` project and click **Build Project**.

5. The project should build successfully.

6. You can now simulate the scenarios provided in the `omnetpp-mesh-tester` project.

For simulation instructions, refer to:  
[https://github.com/LoRaFYP19/omnetpp-mesh-tester?tab=readme-ov-file#how-to-deploy-and-run-simulations](https://github.com/LoRaFYP19/omnetpp-mesh-tester?tab=readme-ov-file#how-to-deploy-and-run-simulations)

---

✅ **Setup Complete**  
