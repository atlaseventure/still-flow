plugins { id("com.android.application") version "9.5.1" }

android {
    namespace = "io.stillflow.runner"
    compileSdk { version = release(37) { minorApiLevel = 2 } }
    buildToolsVersion = "37.0.0"
    ndkVersion = "30.0.16248370"
    defaultConfig {
        applicationId = "io.stillflow.runner"
        minSdk = 29
        targetSdk = 37
        versionCode = 1
        versionName = "1"
        ndk { abiFilters += "arm64-v8a" }
        externalNativeBuild {
            cmake {
                arguments += "-DANDROID_STL=c++_static"
            }
        }
    }
    externalNativeBuild {
        cmake {
            path = file("src/main/cpp/CMakeLists.txt")
            version = "4.2.3"
        }
    }
    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }
    buildTypes {
        release {
            isMinifyEnabled = true
            isShrinkResources = true
            // A local tool: run-as transfers files without shared-storage permissions.
            isDebuggable = true
            signingConfig = signingConfigs.getByName("debug")
            proguardFiles(getDefaultProguardFile("proguard-android-optimize.txt"), "proguard-rules.pro")
        }
    }
    lint {
        warningsAsErrors = true
        abortOnError = true
        disable += "ChromeOsAbiSupport"
    }
}
dependencies {
    implementation(files(providers.gradleProperty("stillflow.aar")
        .getOrElse("../../api/android/build/outputs/aar/stillflow-release.aar")))
}
