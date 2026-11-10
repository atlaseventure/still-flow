plugins {
    id("com.android.library") version "9.5.1"
}

android {
    namespace = "io.stillflow"
    compileSdk {
        version = release(37) { minorApiLevel = 2 }
    }
    buildToolsVersion = "37.0.0"
    ndkVersion = "30.0.16248370"

    defaultConfig {
        minSdk = 26
        ndk { abiFilters += "arm64-v8a" }
        consumerProguardFiles("consumer-rules.pro")
        externalNativeBuild {
            cmake {
                arguments += "-DANDROID_STL=c++_static"
                val jobs = providers.gradleProperty("stillflow.jobs").getOrElse("12")
                require(jobs.toInt() > 0) { "stillflow.jobs must be positive" }
                arguments += listOf("-DCMAKE_JOB_POOLS=stillflow=$jobs", "-DCMAKE_JOB_POOL_COMPILE=stillflow")
                targets += "stillflow_jni"
            }
        }
    }
    externalNativeBuild {
        cmake {
            path = file("CMakeLists.txt")
            version = "4.2.3"
        }
    }
    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }
    lint {
        warningsAsErrors = true
        abortOnError = true
        // The supported Android target is arm64-v8a.
        disable += "ChromeOsAbiSupport"
    }
}

abstract class GenerateNotices : DefaultTask() {
    @get:Inject abstract val fileSystem: FileSystemOperations
    @get:InputDirectory abstract val licenseDirectory: DirectoryProperty
    @get:InputFile abstract val projectLicense: RegularFileProperty
    @get:InputFile abstract val nativeBuildFile: RegularFileProperty
    @get:OutputDirectory abstract val outputDirectory: DirectoryProperty

    @TaskAction fun generate() {
        fileSystem.sync {
            into(outputDirectory)
            from(projectLicense) { into("stillflow") }
            from(licenseDirectory) { into("eigen") }
        }
        val sourceUrl = Regex("URL (https://gitlab.com/libeigen/eigen/[^\\s]+)")
            .find(nativeBuildFile.get().asFile.readText())!!.groupValues[1]
        outputDirectory.file("eigen/SOURCE.txt").get().asFile
            .writeText("Corresponding Eigen source: $sourceUrl\n")
    }
}
val notices = tasks.register<GenerateNotices>("generateNotices") {
    licenseDirectory.set(layout.projectDirectory.dir("../../licenses/eigen"))
    projectLicense.set(layout.projectDirectory.file("../../LICENSE"))
    nativeBuildFile.set(layout.projectDirectory.file("../../CMakeLists.txt"))
    outputDirectory.set(layout.buildDirectory.dir("generated/notices"))
}
androidComponents.onVariants { variant ->
    variant.sources.assets?.addGeneratedSourceDirectory(notices, GenerateNotices::outputDirectory)
}
