This file set contains new files, and changes to existing files, that will enable PhysicsEffects
and several sample apps to be built and run on Android devices. The Android projects are
a mix of Java and C++ code. The code has been tested using Android NDK r5c. Makefiles
are provided for the NDK, and project files for Eclipse. Here are some instructions on how
to update the baseline PhysicsEffects branch of Bullet to support Android using the provided
files:


1. For Windows platforms, download and install Cygwin (if it is not already installed)

2. Download and install the Android SDK.

3. Download and install the Android NDK. (The files provided have been tested
    with NDK r5c.)

4. Set up these new environment variables:

    BULLET_PFX_ROOT_CYGWIN = path to your PhysicsEffects folder, for example:
    BULLET_PFX_ROOT_CYGWIN=/cygdrive/d/tools/Bullet/branches/PhysicsEffects

    CYGWIN_ROOT = path to where Cygwin is installed (Windows-style path)
    ANDROID_NDK_ROOT = path to where the Android NDK is installed
    
5. Download and install Eclipse. Follow procedures on Google's SDK page to
    configure Eclipse to support Android apps. Make sure to set the SDK location
    by using, within Eclipse, Window->Preferences->Android.

6. Import the PhysicsEffects Android project file into Eclipse:

    Within Eclipse, File->Import->General, then choose to import existing
    projects into workspace.

    When the dialog appears, browse to your Physics Effects branch, and to
    folder project/Android. Accept this folder.

    The next dialog will show 9 projects to import: one is PfxLibrary, which
    is PhysicsEffects itself, and 8 sample apps. Import all of these.

7. Unfortunately, when Eclipse imports the files, it won't be able to resolve
    everything. In the Eclipse console, there will be some problems. But they
    are easily fixed. For each PfxApp* package:

    - Select the package in the Eclipse tree control, and right-click select
      Android Tools->Fix Project Properties.

    - From the top menu, select Project->Clean, and clean the project you had
      selected.

    - Now, select the package again, and right-click Refresh.

    - The problems with the package should now be resolved. (But you may
      have to clean/refresh more than once...Eclipse sometimes needs to
      process the project two or three times to actually fix up references and
      generate the needed folders.)

8. Open a Cygwin console. Browse to the PhysicsEffects/project/Android
      folder. Then, for each package, starting with PfxLibrary, go into the
      package folder and run the ndk-build command. For example:

      /PhysicsEffects/project/Android/PfxLibrary> /cygwin/d/tools/android-ndk-r5c/ndk-build

      Build every package using ndk-build.

      (It is possible to configure Eclipse to use ndk-build, but that is somewhat
      tedious to do.)

9. Once everything is build, go back to Eclipse. Select all the packages at
      once and right-click "Refresh."

10. Now you can install the packages on your device. Individually in Eclipse,
      just select a PfxApp* package and right-click run it as an Android app.

      You can also load the app onto the device using the command line adb
      tool (adb push app PfxApp_1_Simple.apk for example)

11. If you run the app via Eclipse, you should see it start on the Android
      device. You can interact with the touch interface:

      - Tap the screen to change scenes
      - Pinch the screen to zoom in/out
      - Drag the screen to rotate the camera
