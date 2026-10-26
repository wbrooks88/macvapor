// MacVapoR: installs the alvrxr OpenXR runtime and sets NotProton or neutron games up to show on
// an Apple Vision Pro through ALVR, or on a PlayStation VR2 plugged into the Mac through Sony's PC
// adapter (the psvr2xr runtime, when the app was built with it; see PSVR2.md). The work per game
// is done by the bundled notproton-vr-setup.sh, or neutron-vr-setup.sh when neutron (the free
// Steam Play tool) is installed; which headset a game shows on is only which runtime the OpenXR
// loader's active_runtime.json names.

import AppKit
import CoreGraphics
import IOKit
import SwiftUI

// macOS 27 SDKs declare a State() macro whose plugin ships only with Xcode, so the Command
// Line Tools cannot expand @State. The alias names the property wrapper type directly.
typealias ViewState<Value> = SwiftUI.State<Value>

enum Paths {
    static let home = FileManager.default.homeDirectoryForCurrentUser
    static let support = home.appending(path: "Library/Application Support/MacVapoR")
    static let runtime = support.appending(path: "runtime")
    static let wineopenxr = support.appending(path: "wineopenxr")
    static let script = support.appending(path: "notproton-vr-setup.sh")
    static let audioHelper = support.appending(path: "MacVapoR Audio.app")
    static let audioPermission = support.appending(path: "audio-permission")
    static let version = support.appending(path: "version")
    static let openComposite = support.appending(path: "opencomposite/openvr_api.dll")
    static let session = home.appending(path: "Library/Application Support/alvrxr/session.json")
    static let height = home.appending(path: "Library/Application Support/alvrxr/height.json")
    static let activeRuntime = home.appending(path: ".config/openxr/1/active_runtime.json")
    static let runner = home.appending(path: "Library/Application Support/notproton/runners/current")
    static let neutron = home.appending(path: "Library/Application Support/neutron/compatibilitytools.d/neutron_proton/files")
    /// wineopenxr for neutron, with the script that sets games up and builds its Mac half here.
    static let neutronKit = support.appending(path: "wineopenxr-neutron")
    static let neutronScript = neutronKit.appending(path: "neutron-vr-setup.sh")
    static let steam = home.appending(path: "Library/Application Support/Steam")
    static let log = home.appending(path: "Library/Logs/alvrxr.log")
    static let alvrLib = runtime.appending(path: "libalvrxr.dylib")
    static let psvr2Lib = runtime.appending(path: "libpsvr2xr.dylib")
    static let psvr2Selftest = runtime.appending(path: "psvr2xr-selftest")
    static let psvr2Config = home.appending(path: "Library/Application Support/psvr2xr/config.json")
    static let psvr2Log = home.appending(path: "Library/Logs/psvr2xr.log")

    // Code is in Contents/Library, data in Contents/Resources.
    static let bundledCode = Bundle.main.bundleURL.appending(path: "Contents/Library/MacVapoR")
    static let bundledData = Bundle.main.resourceURL!
    static let bundledAudioHelper = Bundle.main.bundleURL.appending(path: "Contents/Helpers/MacVapoR Audio.app")
    /// Installs neutron, or hooks it back into Steam; with the node its Steam helper runs on.
    static let bundledNeutronInstaller = bundledData.appending(path: "install-neutron.sh")
    static let bundledNode = bundledData.appending(path: "node/bin")
}

struct Headset: Identifiable {
    let id: String
    let name: String
    var trusted: Bool
}

/// The PS VR2 as macOS sees it: its USB device for tracking and its display for the picture.
enum PSVR2 {
    /// The headset's USB device, as the runtime opens it (src/psvr2/cpsvr2.c).
    static func onUSB() -> Bool {
        let match = IOServiceMatching("IOUSBHostDevice") as NSMutableDictionary
        match["idVendor"] = 0x054C
        match["idProduct"] = 0x0CDE
        let service = IOServiceGetMatchingService(kIOMainPortDefault, match as CFDictionary)
        defer { if service != 0 { IOObjectRelease(service) } }
        return service != 0
    }

    /// A Sense controller connected over Bluetooth (or USB): 0x0E45 left, 0x0E46 right.
    static func controllerConnected(left: Bool) -> Bool {
        let match = IOServiceMatching("IOHIDDevice") as NSMutableDictionary
        match["VendorID"] = 0x054C
        match["ProductID"] = left ? 0x0E45 : 0x0E46
        let service = IOServiceGetMatchingService(kIOMainPortDefault, match as CFDictionary)
        defer { if service != 0 { IOObjectRelease(service) } }
        return service != 0
    }

    enum Display: Equatable { case extended(hz: Double), mirrored, missing }

    /// Found as the runtime finds it (backend_psvr2.m): Sony's EDID vendor id or a 4000-wide mode.
    static func display() -> Display {
        var ids = [CGDirectDisplayID](repeating: 0, count: 16)
        var count: UInt32 = 0
        guard CGGetOnlineDisplayList(16, &ids, &count) == .success else { return .missing }
        var mirrored = false
        for id in ids.prefix(Int(count)) {
            let modes = CGDisplayCopyAllDisplayModes(id, nil) as? [CGDisplayMode] ?? []
            guard CGDisplayVendorNumber(id) == 0x4DD9 || modes.contains(where: { $0.pixelWidth == 4000 }) else { continue }
            if CGDisplayMirrorsDisplay(id) != kCGNullDirectDisplay { mirrored = true; continue }
            return .extended(hz: CGDisplayCopyDisplayMode(id)?.refreshRate ?? 0)
        }
        return mirrored ? .mirrored : .missing
    }

    /// The runtime's config.json, which backend_psvr2.m reads when a game starts.
    struct Settings: Equatable {
        var renderScale = 0.75
        var eyeHeight = 1.6
        var lookaheadMs = 0.0
        var chromatic = true
        var reprojection = true
        var positional = false
        var controllers = true
        var controllerTracking = true
        var eyeTracking = true
        var showGaze = false
        var foveated = true
        // Kept as read, so saving doesn't reset what was set in config.json by hand.
        var foveatedAreaDeg = 40.0
        var foveatedOuterScale = 0.5
        var foveatedInsetScale = 1.0
        var controllerLatencyMs = 25.0
        var controllerThreshold = 170.0
        var controllerSmoothing = 1.0

        static func load() -> Settings {
            var s = Settings()
            guard let data = try? Data(contentsOf: Paths.psvr2Config),
                  let c = try? JSONSerialization.jsonObject(with: data) as? [String: Any] else { return s }
            s.renderScale = c["render_scale"] as? Double ?? s.renderScale
            s.eyeHeight = c["eye_height"] as? Double ?? s.eyeHeight
            s.lookaheadMs = c["lookahead_ms"] as? Double ?? s.lookaheadMs
            s.chromatic = c["chromatic"] as? Bool ?? s.chromatic
            s.reprojection = c["reprojection"] as? Bool ?? s.reprojection
            s.positional = c["positional"] as? Bool ?? s.positional
            s.controllers = c["controllers"] as? Bool ?? s.controllers
            s.controllerTracking = c["controller_tracking"] as? Bool ?? s.controllerTracking
            s.controllerLatencyMs = c["controller_latency_ms"] as? Double ?? s.controllerLatencyMs
            s.controllerThreshold = c["controller_threshold"] as? Double ?? s.controllerThreshold
            s.controllerSmoothing = c["controller_smoothing"] as? Double ?? s.controllerSmoothing
            s.eyeTracking = c["eye_tracking"] as? Bool ?? s.eyeTracking
            s.showGaze = c["show_gaze"] as? Bool ?? s.showGaze
            s.foveated = c["foveated"] as? Bool ?? s.foveated
            s.foveatedAreaDeg = c["foveated_area_deg"] as? Double ?? s.foveatedAreaDeg
            s.foveatedOuterScale = c["foveated_outer_scale"] as? Double ?? s.foveatedOuterScale
            s.foveatedInsetScale = c["foveated_inset_scale"] as? Double ?? s.foveatedInsetScale
            return s
        }

        func save() {
            let c: [String: Any] = ["render_scale": renderScale, "eye_height": eyeHeight, "lookahead_ms": lookaheadMs,
                                    "chromatic": chromatic, "reprojection": reprojection, "positional": positional,
                                    "controllers": controllers, "eye_tracking": eyeTracking, "show_gaze": showGaze,
                                    "foveated": foveated, "foveated_area_deg": foveatedAreaDeg,
                                    "foveated_outer_scale": foveatedOuterScale, "foveated_inset_scale": foveatedInsetScale,
                                    "controller_tracking": controllerTracking, "controller_latency_ms": controllerLatencyMs,
                                    "controller_threshold": controllerThreshold, "controller_smoothing": controllerSmoothing]
            try? FileManager.default.createDirectory(at: Paths.psvr2Config.deletingLastPathComponent(), withIntermediateDirectories: true)
            if let out = try? JSONSerialization.data(withJSONObject: c, options: [.prettyPrinted, .sortedKeys]) {
                try? out.write(to: Paths.psvr2Config, options: .atomic)
            }
        }
    }
}

struct Game: Identifiable {
    enum Kind: String { case openXR = "OpenXR", openVR = "OpenVR", other = "" }
    let id: String
    let name: String
    let dir: URL
    let prefix: URL?
    let exes: [String]
    let kind: Kind
    var configured: Bool
    /// CX_GRAPHICS_BACKEND from the game's launch options, which NotProton's Compatibility
    /// page writes; nil when none is set.
    let graphics: String?
}

@MainActor
final class Model: ObservableObject {
    /// Which headset games show on: which runtime active_runtime.json names.
    enum Target: Equatable { case visionPro, psvr2 }

    @Published var installedVersion: String?
    @Published var target = Model.activeTarget()
    @Published var psvr2USB = false
    @Published var psvr2Display = PSVR2.Display.missing
    @Published var senseLeft = false
    @Published var senseRight = false
    @Published var psvr2Settings = PSVR2.Settings.load()
    @Published var selftest: Process?
    @Published var headsets: [Headset] = []
    /// nil: the real floor (ALVR's LocalFloor); otherwise a fixed eye height in metres (Local).
    @Published var seatedEyeHeight: Double?
    @Published var games: [Game] = []
    @Published var output = ""
    @Published var busy = false
    /// Whether MacVapoR Audio may record games' audio: nil while checking.
    @Published var audioAllowed: Bool?
    @Published var audioChecking = false

    let bundleVersion = Bundle.main.infoDictionary?["CFBundleShortVersionString"] as? String ?? "?"
    var hasRunner: Bool { FileManager.default.fileExists(atPath: Paths.runner.path) }
    /// neutron is installed; games are then set up for it rather than for NotProton.
    var hasNeutron: Bool { FileManager.default.fileExists(atPath: Paths.neutron.appending(path: "bin/wine").path) }
    var bundleHasNeutronKit: Bool { FileManager.default.fileExists(atPath: Paths.bundledData.appending(path: "wineopenxr-neutron").path) }
    /// The Steam Play tool games run with, as the instructions name it.
    var toolName: String { hasNeutron ? "neutron" : "NotProton" }
    /// neutron's start script is Steam.app's executable; a Steam update undoes that.
    var steamHooked: Bool {
        let plist = URL(fileURLWithPath: "/Applications/Steam.app/Contents/Info.plist")
        guard let d = NSDictionary(contentsOf: plist) else { return false }
        return d["CFBundleExecutable"] as? String == "steam_neutron"
    }
    /// Steam Play is on for all games with neutron (the "0" entry of Steam's CompatToolMapping);
    /// neutron's Steam helper can no longer turn it on, and a Steam update can turn it off.
    var steamPlayAll: Bool {
        let config = Paths.home.appending(path: "Library/Application Support/Steam/config/config.vdf")
        guard let text = try? String(contentsOf: config, encoding: .utf8) else { return false }
        return text.range(of: #""CompatToolMapping"\s*\{[^}]*?"0"\s*\{\s*"name"\s*"neutron_proton""#, options: .regularExpression) != nil
    }
    /// A Steam update undid what neutron needs (the hook or Steam Play), and the installer can put it back.
    var steamNeedsRepair: Bool { hasNeutron && canInstallNeutron && (!steamHooked || !steamPlayAll) }
    var canInstallNeutron: Bool { FileManager.default.fileExists(atPath: Paths.bundledNeutronInstaller.path) }
    /// Everything for neutron and the PS VR2 is in place: what the Install Everything button checks.
    var everythingInstalled: Bool {
        installedVersion == bundleVersion && hasNeutron && steamHooked && steamPlayAll && (!bundleHasPSVR2 || hasPSVR2Runtime)
    }
    var hasOpenComposite: Bool { FileManager.default.fileExists(atPath: Paths.openComposite.path) }
    /// This copy of the app carries the PS VR2 runtime, and it has been installed.
    var bundleHasPSVR2: Bool { FileManager.default.fileExists(atPath: Paths.bundledCode.appending(path: "runtime/libpsvr2xr.dylib").path) }
    var hasPSVR2Runtime: Bool { FileManager.default.fileExists(atPath: Paths.psvr2Lib.path) }
    private var timer: Timer?

    init() {
        refresh()
        checkAudioPermission()
        // The self-test's window covers the headset display; it must not outlive the app.
        NotificationCenter.default.addObserver(forName: NSApplication.willTerminateNotification, object: nil, queue: nil) { _ in
            MainActor.assumeIsolated { self.selftest?.terminate() }
        }
        // The PS VR2 comes and goes; its status is cheap to read.
        timer = Timer.scheduledTimer(withTimeInterval: 2, repeats: true) { [weak self] _ in
            Task { @MainActor in
                guard let self, self.target == .psvr2 else { return }
                self.refreshPSVR2()
            }
        }
    }

    // MARK: game audio

    /// MacVapoR Audio taps briefly, which makes macOS ask the first time, and reports back. It is
    /// opened through Launch Services so that the permission is its own, as when games use it.
    func checkAudioPermission() {
        guard FileManager.default.fileExists(atPath: Paths.audioHelper.path) else { return }
        audioChecking = true
        audioAllowed = nil
        try? FileManager.default.removeItem(at: Paths.audioPermission)
        Task.detached {
            let p = Process()
            p.executableURL = URL(fileURLWithPath: "/usr/bin/open")
            p.arguments = ["-W", "-g", "-n", "-a", Paths.audioHelper.path, "--args", "--check", Paths.audioPermission.path]
            try? p.run()
            p.waitUntilExit()
            let result = try? String(contentsOf: Paths.audioPermission, encoding: .utf8)
            await MainActor.run {
                self.audioAllowed = result == "allowed"
                self.audioChecking = false
            }
        }
    }

    func refresh() {
        installedVersion = try? String(contentsOf: Paths.version, encoding: .utf8)
        target = Self.activeTarget()
        loadHeadsets()
        refreshPSVR2()
        games = GameScanner.scan()
    }

    func refreshPSVR2() {
        psvr2USB = PSVR2.onUSB()
        psvr2Display = PSVR2.display()
        senseLeft = PSVR2.controllerConnected(left: true)
        senseRight = PSVR2.controllerConnected(left: false)
    }

    // MARK: runtime

    func installRuntime() {
        do {
            let fm = FileManager.default
            try? fm.removeItem(at: Paths.runtime)
            try? fm.removeItem(at: Paths.wineopenxr)
            try fm.createDirectory(at: Paths.support, withIntermediateDirectories: true)
            try fm.copyItem(at: Paths.bundledCode.appending(path: "runtime"), to: Paths.runtime)
            // wineopenxr's layout, as the setup script and Wine expect it: the PE half is data,
            // the unix half is code.
            try fm.createDirectory(at: Paths.wineopenxr, withIntermediateDirectories: true)
            try fm.copyItem(at: Paths.bundledData.appending(path: "wineopenxr/x86_64-windows"),
                            to: Paths.wineopenxr.appending(path: "x86_64-windows"))
            try fm.copyItem(at: Paths.bundledData.appending(path: "wineopenxr/wineopenxr64.json"),
                            to: Paths.wineopenxr.appending(path: "wineopenxr64.json"))
            try fm.copyItem(at: Paths.bundledCode.appending(path: "wineopenxr/aarch64-unix"),
                            to: Paths.wineopenxr.appending(path: "aarch64-unix"))
            try? fm.removeItem(at: Paths.script)
            try fm.copyItem(at: Paths.bundledData.appending(path: "notproton-vr-setup.sh"), to: Paths.script)
            // neutron's kit is data: its script builds wineopenxr's Mac half beside it on first use.
            try? fm.removeItem(at: Paths.neutronKit)
            if bundleHasNeutronKit {
                try fm.copyItem(at: Paths.bundledData.appending(path: "wineopenxr-neutron"), to: Paths.neutronKit)
            }
            try? fm.removeItem(at: Paths.audioHelper)
            try fm.copyItem(at: Paths.bundledAudioHelper, to: Paths.audioHelper)

            if !fm.fileExists(atPath: Paths.session.path) {
                try fm.createDirectory(at: Paths.session.deletingLastPathComponent(), withIntermediateDirectories: true)
                try fm.copyItem(at: Paths.bundledData.appending(path: "session-template.json"), to: Paths.session)
            }
            try writeActiveRuntime(hasPSVR2Runtime ? target : .visionPro)
            try bundleVersion.write(to: Paths.version, atomically: true, encoding: .utf8)
            Self.clearQuarantine(Paths.support)
            output = "Runtime \(bundleVersion) installed."
            checkAudioPermission()
        } catch {
            output = "Install failed: \(error.localizedDescription)"
        }
        refresh()
    }

    /// The one button for a new Mac: the runtime, the PS VR2 as the headset when this app has it,
    /// and neutron with its Steam hook (whose installer quits and restarts Steam).
    func installEverything() {
        installRuntime()
        if hasPSVR2Runtime { setTarget(.psvr2) }
        guard canInstallNeutron, !hasNeutron || !steamHooked || !steamPlayAll else {
            refresh()
            return
        }
        runNeutronInstaller(hasNeutron ? "Putting neutron back into Steam after a Steam update; Steam quits and starts again…"
                                       : "Installing neutron: downloading about 260 MB, then Steam quits and starts again. This takes a few minutes…")
    }

    /// Runs the bundled neutron installer (which also only repairs Steam's hook and Steam Play when
    /// neutron is there), showing its output; then, if it worked, done.
    func runNeutronInstaller(_ message: String, then done: (() -> Void)? = nil) {
        var env = ProcessInfo.processInfo.environment
        if FileManager.default.fileExists(atPath: Paths.bundledNode.appending(path: "node").path) {
            env["NODE_DIR"] = Paths.bundledNode.path
        }
        busy = true
        output = message
        Task.detached {
            let p = Process()
            p.executableURL = URL(fileURLWithPath: "/bin/bash")
            p.arguments = [Paths.bundledNeutronInstaller.path]
            p.environment = env
            let pipe = Pipe()
            p.standardOutput = pipe
            p.standardError = pipe
            var text: String
            do {
                try p.run()
                let data = pipe.fileHandleForReading.readDataToEndOfFile()
                p.waitUntilExit()
                // neutron's installer colours its lines; the output box shows plain text.
                text = String(decoding: data, as: UTF8.self)
                    .replacingOccurrences(of: "\u{1B}\\[[0-9;]*m", with: "", options: .regularExpression)
                if p.terminationStatus != 0 { text += "\n(neutron install failed, status \(p.terminationStatus))" }
            } catch {
                text = "Could not run the neutron installer: \(error.localizedDescription)"
            }
            let ok = text.range(of: "(neutron install failed") == nil && !text.hasPrefix("Could not run")
            let shown = text
            await MainActor.run {
                self.output = shown
                self.busy = false
                self.refresh()
                if ok, let done { done() }
            }
        }
    }

    /// Copies out of an app that came from a download keep its quarantine flag, and the setup
    /// script's cp carries it into every game's prefix. These files are part of the notarized
    /// app, so they are installed as trusted, the way an installer would.
    static func clearQuarantine(_ dir: URL) {
        let fm = FileManager.default
        var paths = [dir.path]
        if let walker = fm.enumerator(atPath: dir.path) {
            for case let rel as String in walker { paths.append(dir.appending(path: rel).path) }
        }
        for path in paths { removexattr(path, "com.apple.quarantine", XATTR_NOFOLLOW) }
    }

    static func activeTarget() -> Target {
        let text = (try? String(contentsOf: Paths.activeRuntime, encoding: .utf8)) ?? ""
        return text.contains("libpsvr2xr.dylib") ? .psvr2 : .visionPro
    }

    private func writeActiveRuntime(_ target: Target) throws {
        let fm = FileManager.default
        let dir = Paths.activeRuntime.deletingLastPathComponent()
        try fm.createDirectory(at: dir, withIntermediateDirectories: true)
        // Another runtime's manifest is kept beside ours rather than lost.
        if let old = try? String(contentsOf: Paths.activeRuntime, encoding: .utf8),
           !old.contains("libalvrxr.dylib"), !old.contains("libpsvr2xr.dylib") {
            try? old.write(to: dir.appending(path: "active_runtime.json.before-macvapor"), atomically: true, encoding: .utf8)
        }
        let (name, lib) = target == .psvr2 ? ("psvr2xr", Paths.psvr2Lib.path) : ("alvrxr", Paths.alvrLib.path)
        let manifest = """
            {
                "file_format_version": "1.0.0",
                "runtime": {
                    "name": "\(name)",
                    "library_path": "\(lib)"
                }
            }

            """
        try manifest.write(to: Paths.activeRuntime, atomically: true, encoding: .utf8)
    }

    /// Points every set-up game at one headset. Games pick it up the next time they start.
    func setTarget(_ new: Target) {
        do {
            try writeActiveRuntime(new)
            target = new
            refreshPSVR2()
            output = new == .psvr2 ? "Games now show on the PS VR2." : "Games now show on the Vision Pro through ALVR."
        } catch {
            output = "Could not switch headsets: \(error.localizedDescription)"
        }
    }

    // MARK: PS VR2

    /// With neutron, Play does the setup too: a game not started yet is started once so neutron
    /// makes its prefix, and one with a prefix is set up first and then started.
    func playOrSetUp(_ game: Game, exe: String) {
        // A Steam update may have undone neutron's hook or Steam Play; put them back first.
        if steamNeedsRepair {
            runNeutronInstaller("A Steam update undid neutron's setup; putting it back (Steam quits and starts again)…") { [weak self] in
                guard let self, !self.steamNeedsRepair else { return }
                self.playOrSetUp(game, exe: exe)
            }
            return
        }
        if game.configured || game.prefix == nil || exe.isEmpty {
            play(game)
            if game.prefix == nil {
                output = "Starting \(game.name) once so neutron can prepare it. Quit it, then press Play again for VR."
            }
            return
        }
        setUp(game, exe: exe, undo: false) { [weak self] in
            guard let self, let set = self.games.first(where: { $0.id == game.id }) else { return }
            self.play(set)
        }
    }

    /// Starts a set-up game through Steam, which runs it with NotProton as its compatibility tool.
    func play(_ game: Game) {
        if steamNeedsRepair {
            runNeutronInstaller("A Steam update undid neutron's setup; putting it back (Steam quits and starts again)…") { [weak self] in
                guard let self, !self.steamNeedsRepair else { return }
                self.play(game)
            }
            return
        }
        var notes: [String] = []
        if target == .psvr2 {
            psvr2Settings.save()
            refreshPSVR2()
            if !psvr2USB { notes.append("the PS VR2 is not on USB, so there will be no head tracking") }
            switch psvr2Display {
            case .missing: notes.append("no PS VR2 display was found, so nothing will show in the headset")
            case .mirrored: notes.append("the PS VR2 display mirrors another one; set it to extend the desktop")
            case .extended: break
            }
        }
        NSWorkspace.shared.open(URL(string: "steam://rungameid/\(game.id)")!)
        output = "Starting \(game.name) through Steam." + (notes.isEmpty ? "" : " Note: " + notes.joined(separator: "; ") + ".")
    }

    enum SelftestMode { case normal, slow, calibrateEyes, foveated, captureControllers }

    func startSelftest(_ mode: SelftestMode) {
        guard selftest == nil else { return }
        let p = Process()
        p.executableURL = Paths.psvr2Selftest
        switch mode {
        case .normal: p.arguments = []
        case .slow: p.arguments = ["--slow", "2"]
        case .calibrateEyes: p.arguments = ["--calibrate-eyes"]
        case .foveated: p.arguments = ["--foveated"]
        case .captureControllers: p.arguments = ["--capture-controllers"]
        }
        p.terminationHandler = { _ in Task { @MainActor in self.selftestEnded() } }
        do {
            try p.run()
            selftest = p
            // The self-test takes the keyboard for Esc if macOS lets it; when it doesn't (it won't
            // take focus from the app that started it), Esc reaches MacVapoR, so stop it from here.
            if let app = NSRunningApplication(processIdentifier: p.processIdentifier) {
                NSApp.yieldActivation(to: app)
            }
            escMonitor = NSEvent.addLocalMonitorForEvents(matching: .keyDown) { [weak self] e in
                guard e.keyCode == 53, let self, self.selftest != nil else { return e }
                self.stopSelftest()
                return nil
            }
            switch mode {
            case .calibrateEyes:
                output = "Eye calibration: put the headset on and look at each dot until the next one appears (13 dots, about a minute). Press Stop when the test room comes back."
            case .foveated:
                output = "Foveated test: the test room, with the sharp area tinted orange. It should follow your eyes, with a yellow ring where you look."
            case .captureControllers:
                output = "Controller capture: the test room, while camera frames are saved to the PS VR2 Controller Capture folder on your Desktop. Hold both controllers in front of you and move them around slowly for 20 seconds, raising them over your head too, then press Stop and send Claude that folder as a zip."
            default:
                output = "Self-test running: put the headset on. You should see a floor grid and four pillars: red ahead, green right, blue behind, yellow left."
            }
            output += " To end it, press Stop or Esc, hold the headset button for 5 seconds, or quit MacVapoR; it also ends by itself after 10 minutes."
        } catch {
            output = "Could not start the self-test: \(error.localizedDescription)"
        }
    }

    private var escMonitor: Any?

    private func selftestEnded() {
        selftest = nil
        if let m = escMonitor { NSEvent.removeMonitor(m) }
        escMonitor = nil
    }

    /// Asks the self-test to quit, then kills it if it hasn't after two seconds.
    func stopSelftest() {
        guard let p = selftest else { return }
        p.terminate()
        DispatchQueue.main.asyncAfter(deadline: .now() + 2) {
            if p.isRunning { kill(p.processIdentifier, SIGKILL) }
        }
    }

    // MARK: headsets

    func loadHeadsets() {
        guard let data = try? Data(contentsOf: Paths.session),
              let json = try? JSONSerialization.jsonObject(with: data) as? [String: Any] else {
            headsets = []
            return
        }
        seatedEyeHeight = (try? Data(contentsOf: Paths.height))
            .flatMap { try? JSONSerialization.jsonObject(with: $0) as? [String: Any] }
            .flatMap { $0["eye_height"] as? Double }
        guard let clients = json["client_connections"] as? [String: [String: Any]] else {
            headsets = []
            return
        }
        headsets = clients.map { id, c in
            Headset(id: id, name: c["display_name"] as? String ?? id, trusted: c["trusted"] as? Bool ?? false)
        }.sorted { $0.id < $1.id }
    }

    /// The real floor for standing play, or the eyes put at a fixed height for seated play. The
    /// runtime applies it, from height.json, when a game starts: ALVR's own recentering does not
    /// run with the visionOS client, so ALVR is left on the real floor.
    func setSeatedEyeHeight(_ height: Double?) {
        if let height {
            try? JSONSerialization.data(withJSONObject: ["eye_height": height]).write(to: Paths.height, options: .atomic)
        } else {
            try? FileManager.default.removeItem(at: Paths.height)
        }
        guard let data = try? Data(contentsOf: Paths.session),
              var json = try? JSONSerialization.jsonObject(with: data) as? [String: Any],
              var settings = json["session_settings"] as? [String: Any],
              var headset = settings["headset"] as? [String: Any],
              var mode = headset["position_recentering_mode"] as? [String: Any] else { return }
        mode["variant"] = "LocalFloor"
        headset["position_recentering_mode"] = mode
        settings["headset"] = headset
        json["session_settings"] = settings
        if let out = try? JSONSerialization.data(withJSONObject: json, options: [.prettyPrinted, .sortedKeys]) {
            try? out.write(to: Paths.session, options: .atomic)
        }
        loadHeadsets()
    }

    func setTrusted(_ id: String, _ trusted: Bool) {
        guard let data = try? Data(contentsOf: Paths.session),
              var json = try? JSONSerialization.jsonObject(with: data) as? [String: Any],
              var clients = json["client_connections"] as? [String: [String: Any]],
              clients[id] != nil else { return }
        clients[id]?["trusted"] = trusted
        json["client_connections"] = clients
        if let out = try? JSONSerialization.data(withJSONObject: json, options: [.prettyPrinted, .sortedKeys]) {
            try? out.write(to: Paths.session, options: .atomic)
        }
        loadHeadsets()
    }

    // MARK: games

    /// Runs the setup script for one game; `then` runs once it has succeeded.
    func setUp(_ game: Game, exe: String, undo: Bool, then: (@MainActor () -> Void)? = nil) {
        var env = ProcessInfo.processInfo.environment
        env["GFX"] = "dxmt"
        env["WOX"] = Paths.wineopenxr.path
        env["ALVRXR_LIB"] = Paths.alvrLib.path
        env["TARGET"] = target == .psvr2 ? "psvr2" : "alvr"
        // The Command Line Tools' clang, which neutron-vr-setup.sh builds with, and Homebrew's bin.
        env["PATH"] = "/usr/bin:/bin:/usr/sbin:/sbin:/opt/homebrew/bin"
        let neutron = hasNeutron
        if neutron && !FileManager.default.fileExists(atPath: Paths.neutronScript.path) {
            output = bundleHasNeutronKit ? "Press Reinstall above to add neutron support." : "This copy of MacVapoR was built without neutron support; see PSVR2.md."
            return
        }
        if game.kind == .openVR && !undo {
            guard hasOpenComposite else {
                output = "\(game.name) uses OpenVR: download OpenComposite first."
                return
            }
            env["OC"] = Paths.openComposite.path
        }
        var args = [neutron ? Paths.neutronScript.path : Paths.script.path]
        if undo { args.append("--undo") }
        args += [game.id, exe]
        busy = true
        output = neutron ? "Setting up \(game.name) for neutron (the first time builds a small part, which takes a few seconds)…" : ""
        Task.detached {
            let p = Process()
            p.executableURL = URL(fileURLWithPath: "/bin/bash")
            p.arguments = args
            p.environment = env
            let pipe = Pipe()
            p.standardOutput = pipe
            p.standardError = pipe
            let text: String
            var ok = false
            do {
                try p.run()
                let data = pipe.fileHandleForReading.readDataToEndOfFile()
                p.waitUntilExit()
                ok = p.terminationStatus == 0
                let status = ok ? "" : "\n(failed, status \(p.terminationStatus))"
                text = String(decoding: data, as: UTF8.self) + status
            } catch {
                text = "Could not run the setup script: \(error.localizedDescription)"
            }
            await MainActor.run { [ok] in
                self.output = text
                self.busy = false
                self.refresh()
                if ok { then?() }
            }
        }
    }

    // MARK: OpenComposite

    func downloadOpenComposite() {
        let url = URL(string: "https://znix.xyz/OpenComposite/download.php?arch=x64&branch=openxr")!
        busy = true
        output = "Downloading OpenComposite…"
        Task {
            do {
                let (file, _) = try await URLSession.shared.download(from: url)
                let data = try Data(contentsOf: file)
                guard OpenComposite.looksValid(data) else {
                    throw NSError(domain: "MacVapoR", code: 1,
                                  userInfo: [NSLocalizedDescriptionKey: "the download is not a 64-bit OpenComposite DLL"])
                }
                let fm = FileManager.default
                try fm.createDirectory(at: Paths.openComposite.deletingLastPathComponent(), withIntermediateDirectories: true)
                try data.write(to: Paths.openComposite, options: .atomic)
                Model.clearQuarantine(Paths.openComposite.deletingLastPathComponent())
                output = "OpenComposite downloaded (\(data.count / 1024) KB)."
            } catch {
                output = "OpenComposite download failed: \(error.localizedDescription)"
            }
            busy = false
        }
    }
}

enum OpenComposite {
    /// A 64-bit Windows DLL that implements OpenVR's compositor interface.
    static func looksValid(_ d: Data) -> Bool {
        guard d.count > 0x1000, d[0] == 0x4D, d[1] == 0x5A else { return false }
        let pe = Int(d[0x3C]) | Int(d[0x3D]) << 8 | Int(d[0x3E]) << 16 | Int(d[0x3F]) << 24
        guard pe + 6 < d.count, d[pe] == 0x50, d[pe + 1] == 0x45, d[pe + 4] == 0x64, d[pe + 5] == 0x86 else { return false }
        return d.range(of: Data("IVRCompositor_".utf8)) != nil
    }
}

enum GameScanner {
    // Executables that are never the game.
    static let notGames = ["unitycrashhandler", "unins", "vcredist", "dxsetup", "crashreport", "vconsole", "vrstartup",
                           "unrealcefsubprocess", "ue4prereqsetup", "ueprereqsetup"]

    static func libraries() -> [URL] {
        var libs = [Paths.steam]
        let vdf = Paths.steam.appending(path: "steamapps/libraryfolders.vdf")
        if let text = try? String(contentsOf: vdf, encoding: .utf8) {
            for line in text.split(separator: "\n") where line.contains("\"path\"") {
                let parts = line.split(separator: "\"")
                if parts.count >= 4 {
                    // Steam lists its own library here too; compare paths, as URLs differ by a trailing slash.
                    let url = URL(fileURLWithPath: String(parts[3]).replacingOccurrences(of: "\\\\", with: "\\"))
                    if !libs.contains(where: { $0.standardizedFileURL.path == url.standardizedFileURL.path }) {
                        libs.append(url)
                    }
                }
            }
        }
        return libs
    }

    static func value(_ key: String, in acf: String) -> String? {
        for line in acf.split(separator: "\n") {
            let parts = line.split(separator: "\"")
            if parts.count >= 4, parts[1] == key { return String(parts[3]) }
        }
        return nil
    }

    /// Launch options per app ID from the most recently used Steam account's localconfig.vdf:
    /// UserLocalConfigStore > Software > Valve > Steam > apps > <appid> > LaunchOptions.
    static func launchOptions() -> [String: String] {
        let fm = FileManager.default
        let userdata = Paths.steam.appending(path: "userdata")
        let configs = ((try? fm.contentsOfDirectory(atPath: userdata.path)) ?? []).map {
            userdata.appending(path: "\($0)/config/localconfig.vdf")
        }.filter { fm.fileExists(atPath: $0.path) }
        func modified(_ url: URL) -> Date {
            (try? fm.attributesOfItem(atPath: url.path)[.modificationDate] as? Date) ?? .distantPast
        }
        guard let newest = configs.max(by: { modified($0) < modified($1) }),
              let text = try? String(contentsOf: newest, encoding: .utf8) else { return [:] }
        var path: [String] = []
        var pending: String?
        var options: [String: String] = [:]
        for raw in text.split(separator: "\n") {
            let line = raw.trimmingCharacters(in: .whitespaces)
            if line == "{" {
                path.append(pending ?? "")
                pending = nil
            } else if line == "}" {
                _ = path.popLast()
            } else {
                let parts = line.split(separator: "\"", omittingEmptySubsequences: false)
                if parts.count >= 5 {
                    let key = String(parts[1]), value = String(parts[3])
                    if key == "LaunchOptions", path.count >= 2, path[path.count - 2].lowercased() == "apps" {
                        options[path[path.count - 1]] = value
                    }
                } else if parts.count >= 3 {
                    pending = String(parts[1])
                }
            }
        }
        return options
    }

    static func graphics(in options: String?) -> String? {
        guard let options else { return nil }
        for word in options.split(separator: " ") where word.hasPrefix("CX_GRAPHICS_BACKEND=") {
            return String(word.dropFirst("CX_GRAPHICS_BACKEND=".count))
        }
        return nil
    }

    static func scan() -> [Game] {
        let fm = FileManager.default
        var games: [Game] = []
        let options = launchOptions()
        for lib in libraries() {
            let apps = lib.appending(path: "steamapps")
            guard let items = try? fm.contentsOfDirectory(atPath: apps.path) else { continue }
            for item in items where item.hasPrefix("appmanifest_") && item.hasSuffix(".acf") {
                guard let acf = try? String(contentsOf: apps.appending(path: item), encoding: .utf8),
                      let id = value("appid", in: acf), let name = value("name", in: acf),
                      let installdir = value("installdir", in: acf) else { continue }
                if id == "228980" || id == "250820" { continue }   // Steamworks redistributables, SteamVR
                if games.contains(where: { $0.id == id }) { continue }
                let dir = apps.appending(path: "common/\(installdir)")
                var exes: [String] = []
                var kind = Game.Kind.other
                if let walker = fm.enumerator(at: dir, includingPropertiesForKeys: nil) {
                    for case let url as URL in walker {
                        // Unreal keeps openvr_api.dll seven levels down, in Engine/Binaries/ThirdParty/OpenVR/....
                        if walker.level > 8 { walker.skipDescendants(); continue }
                        let file = url.lastPathComponent.lowercased()
                        if file == "openvr_api.dll" { kind = .openVR }
                        if file == "openxr_loader.dll" && kind == .other { kind = .openXR }
                        if file.hasSuffix(".exe") && !notGames.contains(where: { file.hasPrefix($0) }) {
                            exes.append(String(url.path.dropFirst(dir.path.count + 1)))
                        }
                    }
                }
                exes.sort { ($0.split(separator: "/").count, $0) < ($1.split(separator: "/").count, $1) }
                let prefixes = [apps.appending(path: "compatdata/\(id)/pfx"),
                                Paths.steam.appending(path: "steamapps/compatdata/\(id)/pfx")]
                let prefix = prefixes.first { fm.fileExists(atPath: $0.appending(path: "system.reg").path) }
                // NotProton's setup stages wineopenxr for Steam; neutron's only puts it in system32.
                let configured = prefix.map {
                    fm.fileExists(atPath: $0.appending(path: "drive_c/Program Files (x86)/Steam/aarch64-windows/wineopenxr.dll").path)
                        || fm.fileExists(atPath: $0.appending(path: "drive_c/windows/system32/wineopenxr64.json").path)
                } ?? false
                games.append(Game(id: id, name: name, dir: dir, prefix: prefix, exes: exes, kind: kind,
                                  configured: configured, graphics: graphics(in: options[id])))
            }
        }
        return games.sorted { $0.name.localizedCaseInsensitiveCompare($1.name) == .orderedAscending }
    }
}

struct ContentView: View {
    @EnvironmentObject var model: Model
    @ViewState private var vrOnly = true

    var body: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: 20) {
                runtimeSection
                headsetSection
                gamesSection
                if !model.output.isEmpty {
                    Text(model.output)
                        .font(.system(.caption, design: .monospaced))
                        .textSelection(.enabled)
                        .padding(8)
                        .frame(maxWidth: .infinity, alignment: .leading)
                        .background(.quaternary, in: .rect(cornerRadius: 6))
                }
            }
            .padding(20)
        }
        .disabled(model.busy)
        .toolbar {
            Button("Refresh", systemImage: "arrow.clockwise") { model.refresh() }
            Button("Runtime Log", systemImage: "doc.text") {
                NSWorkspace.shared.open(model.target == .psvr2 ? Paths.psvr2Log : Paths.log)
            }
        }
        .onChange(of: model.psvr2Settings) { _, s in s.save() }
    }

    private var runtimeSection: some View {
        GroupBox("1. Runtime") {
            VStack(alignment: .leading, spacing: 8) {
                if model.canInstallNeutron && !model.everythingInstalled {
                    HStack {
                        Text("New here? This installs the runtime and neutron (the free tool Steam runs Windows games with), and picks the PS VR2. Steam must be installed and logged in; it restarts once.")
                            .font(.callout)
                        Spacer()
                        Button("Install Everything") { model.installEverything() }
                            .buttonStyle(.borderedProminent)
                    }
                }
                if model.hasNeutron {
                    Label("Games run with neutron.", systemImage: "checkmark.circle").foregroundStyle(.secondary)
                } else if !model.hasRunner {
                    Label("No Steam Play tool found. Install neutron (free, see PSVR2.md) or NotProton, and run a game with it once.",
                          systemImage: "exclamationmark.triangle")
                        .foregroundStyle(.orange)
                }
                audioStatus
                HStack {
                    if let v = model.installedVersion {
                        Text(v == model.bundleVersion ? "alvrxr \(v) is installed." : "alvrxr \(v) is installed; this is \(model.bundleVersion).")
                    } else {
                        Text("Not installed.")
                    }
                    Spacer()
                    Button(model.installedVersion == model.bundleVersion ? "Reinstall" : "Install") { model.installRuntime() }
                }
                if model.installedVersion != nil {
                    targetPicker
                }
            }
            .frame(maxWidth: .infinity, alignment: .leading)
            .padding(4)
        }
    }

    @ViewBuilder private var audioStatus: some View {
        if model.installedVersion == nil {
            EmptyView()
        } else if model.audioChecking {
            Label("Game audio: checking. If macOS asks whether MacVapoR Audio may record audio, allow it.",
                  systemImage: "speaker.wave.2")
                .foregroundStyle(.secondary)
        } else if model.audioAllowed == true {
            Label("Game audio: allowed", systemImage: "checkmark.circle").foregroundStyle(.secondary)
        } else if model.audioAllowed == false {
            HStack {
                Label("Game audio is not allowed: turn on MacVapoR Audio under Screen & System Audio Recording.",
                      systemImage: "speaker.slash")
                    .foregroundStyle(.orange)
                Button("Open Settings") {
                    NSWorkspace.shared.open(URL(string: "x-apple.systempreferences:com.apple.preference.security?Privacy_AudioCapture")!)
                }
                Button("Check Again") { model.checkAudioPermission() }
            }
        }
    }

    @ViewBuilder private var targetPicker: some View {
        HStack {
            Picker("Show games on", selection: Binding(get: { model.target }, set: { model.setTarget($0) })) {
                Text("Apple Vision Pro").tag(Model.Target.visionPro)
                Text("PlayStation VR2").tag(Model.Target.psvr2)
            }
            .pickerStyle(.segmented)
            .frame(maxWidth: 420)
            .disabled(!model.hasPSVR2Runtime && model.target == .visionPro)
        }
        if !model.hasPSVR2Runtime {
            Text(model.bundleHasPSVR2 ? "Reinstall to add the PS VR2 runtime." : "This copy of MacVapoR was built without the PS VR2 runtime; see PSVR2.md.")
                .font(.caption)
                .foregroundStyle(.secondary)
        }
    }

    @ViewBuilder private var headsetSection: some View {
        if model.target == .psvr2 {
            psvr2Section
        } else {
            visionProSection
        }
    }

    private var psvr2Section: some View {
        GroupBox("2. PlayStation VR2") {
            VStack(alignment: .leading, spacing: 8) {
                Text("Plug the headset into Sony's PC adapter, and the adapter into the Mac by DisplayPort and USB. Set its display to extend the desktop at 90 Hz in System Settings > Displays.")
                    .font(.callout)
                    .foregroundStyle(.secondary)
                status(model.psvr2USB, "On USB (head tracking)", "Not on USB: check the adapter's USB cable and that the headset is on")
                switch model.psvr2Display {
                case .extended(let hz):
                    status(true, String(format: "Display connected at %.0f Hz%@", hz, hz > 100 ? " (90 Hz is easier on a MacBook)" : ""), "")
                case .mirrored:
                    status(false, "", "The PS VR2 display mirrors another one: set it to extend the desktop in Displays")
                case .missing:
                    status(false, "", "No PS VR2 display: check the DisplayPort cable and the adapter's power")
                }
                if model.psvr2Settings.controllers {
                    status(model.senseLeft && model.senseRight,
                           "Both Sense controllers connected",
                           model.senseLeft || model.senseRight
                               ? "Only the \(model.senseLeft ? "left" : "right") Sense controller is connected"
                               : "No Sense controllers: press PS to wake them, or pair them (see below)")
                    if !(model.senseLeft && model.senseRight) {
                        Text("To pair one, hold its PS button and Create (left) or Options (right) until its light blinks, then connect it in System Settings > Bluetooth.")
                            .font(.caption).foregroundStyle(.secondary)
                    }
                }
                HStack {
                    Text("Self-test: a test room in the headset, without a game.").foregroundStyle(.secondary)
                    Spacer()
                    if model.selftest == nil {
                        Button("Run") { model.startSelftest(.normal) }
                        Button("Run at half rate") { model.startSelftest(.slow) }
                    } else {
                        Button("Stop") { model.stopSelftest() }
                    }
                }
                HStack {
                    Text("Eye tracking: calibrate it once (it is saved), then check it follows your eyes.").foregroundStyle(.secondary)
                    Spacer()
                    if model.selftest == nil {
                        Button("Calibrate eyes") { model.startSelftest(.calibrateEyes) }
                        Button("Foveated test") { model.startSelftest(.foveated) }
                    }
                }
                HStack {
                    Text("Controller tracking: the balls in the test room should follow your hands. If they don't, capture what the cameras see.").foregroundStyle(.secondary)
                    Spacer()
                    if model.selftest == nil {
                        Button("Capture controllers") { model.startSelftest(.captureControllers) }
                    }
                }
                Divider()
                HStack {
                    Text("Render scale")
                    Slider(value: $model.psvr2Settings.renderScale, in: 0.4...1.2, step: 0.05).frame(maxWidth: 260)
                    Text(String(format: "%.2f", model.psvr2Settings.renderScale)).monospacedDigit()
                    Text("lower it if games stutter").font(.caption).foregroundStyle(.secondary)
                }
                HStack {
                    Text("Eye height")
                    Slider(value: $model.psvr2Settings.eyeHeight, in: 1.0...2.0, step: 0.05).frame(maxWidth: 260)
                    Text(String(format: "%.2f m", model.psvr2Settings.eyeHeight)).monospacedDigit()
                }
                HStack {
                    Text("Extra prediction")
                    Slider(value: $model.psvr2Settings.lookaheadMs, in: -10...20, step: 1).frame(maxWidth: 260)
                    Text(String(format: "%+.0f ms", model.psvr2Settings.lookaheadMs)).monospacedDigit()
                }
                Toggle("Smooth head turns (reprojection)", isOn: $model.psvr2Settings.reprojection)
                Toggle("Per-colour lens correction", isOn: $model.psvr2Settings.chromatic)
                Toggle("Room-scale position (experimental)", isOn: $model.psvr2Settings.positional)
                Toggle("Sense controllers", isOn: $model.psvr2Settings.controllers)
                Toggle("Track controller position with the headset cameras", isOn: $model.psvr2Settings.controllerTracking)
                    .disabled(!model.psvr2Settings.controllers)
                HStack {
                    Text("Controller smoothing")
                    Slider(value: $model.psvr2Settings.controllerSmoothing, in: 0...3, step: 0.25).frame(maxWidth: 260)
                    Text(model.psvr2Settings.controllerSmoothing == 0 ? "off" : String(format: "%.2f", model.psvr2Settings.controllerSmoothing))
                        .monospacedDigit()
                    Text("higher: steadier when still, a little more lag").font(.caption).foregroundStyle(.secondary)
                }
                .disabled(!model.psvr2Settings.controllerTracking)
                Toggle("Eye tracking", isOn: $model.psvr2Settings.eyeTracking)
                Toggle("Show where I look (a yellow ring)", isOn: $model.psvr2Settings.showGaze)
                Toggle("Foveated rendering, in games that support it (quad views)", isOn: $model.psvr2Settings.foveated)
                Text("Takes effect the next time a game starts. The headset's Fn button recenters the view and the controllers; hold it for 2 seconds to calibrate the eye tracker. A controller's PS button points it forward again.")
                    .font(.caption).foregroundStyle(.secondary)
            }
            .frame(maxWidth: .infinity, alignment: .leading)
            .padding(4)
        }
    }

    private func status(_ ok: Bool, _ good: String, _ bad: String) -> some View {
        Label(ok ? good : bad, systemImage: ok ? "checkmark.circle" : "exclamationmark.triangle")
            .foregroundStyle(ok ? Color.secondary : Color.orange)
    }

    private var visionProSection: some View {
        GroupBox("2. Headsets") {
            VStack(alignment: .leading, spacing: 8) {
                Text("Open ALVR on the Vision Pro while a set-up game is running, and it shows up here. Trust it, then restart the game.")
                    .font(.callout)
                    .foregroundStyle(.secondary)
                if model.headsets.isEmpty {
                    Text("No headsets yet.")
                }
                Picker("Height", selection: Binding(get: { model.seatedEyeHeight == nil ? 0 : 1 },
                                                    set: { model.setSeatedEyeHeight($0 == 0 ? nil : 1.5) })) {
                    Text("Real floor (standing)").tag(0)
                    Text("Seated: fixed eye height").tag(1)
                }
                .pickerStyle(.segmented)
                .frame(maxWidth: 420)
                if let height = model.seatedEyeHeight {
                    HStack {
                        Slider(value: Binding(get: { height }, set: { model.setSeatedEyeHeight(($0 * 20).rounded() / 20) }),
                               in: 1.0...2.0)
                            .frame(maxWidth: 300)
                        Text(String(format: "%.2f m", height)).monospacedDigit()
                    }
                }
                Text("Takes effect the next time a game starts.").font(.caption).foregroundStyle(.secondary)
                ForEach(model.headsets) { h in
                    Toggle(isOn: Binding(get: { h.trusted }, set: { model.setTrusted(h.id, $0) })) {
                        HStack {
                            Text(h.name).bold()
                            Text(h.id).foregroundStyle(.secondary)
                        }
                    }
                }
            }
            .frame(maxWidth: .infinity, alignment: .leading)
            .padding(4)
        }
    }

    private var gamesHelp: String {
        let before = model.hasNeutron
            ? "Close the game before setting it up."
            : "In NotProton, set the game's graphics to DXMT. Close the game before setting it up."
        let after = model.target == .psvr2
            ? "Then put the headset on, wake the Sense controllers, and press Play. The controllers turn but don't track position, so games where you reach for things work less well."
            : "Then open ALVR on the Vision Pro with the controllers awake, and press Play."
        return before + " " + after
    }

    private var gamesSection: some View {
        GroupBox("3. Games") {
            VStack(alignment: .leading, spacing: 10) {
                Text(verbatim: gamesHelp)
                    .font(.callout)
                    .foregroundStyle(.secondary)
                HStack {
                    Toggle("VR games only", isOn: $vrOnly)
                    Spacer()
                    if model.hasOpenComposite {
                        Label("OpenComposite ready", systemImage: "checkmark.circle").foregroundStyle(.secondary)
                    }
                    Button(model.hasOpenComposite ? "Update OpenComposite" : "Download OpenComposite") {
                        model.downloadOpenComposite()
                    }
                }
                ForEach(model.games.filter { !vrOnly || $0.kind != .other }) { game in
                    GameRow(game: game)
                }
            }
            .frame(maxWidth: .infinity, alignment: .leading)
            .padding(4)
        }
    }
}

struct GameRow: View {
    @EnvironmentObject var model: Model
    let game: Game
    @ViewState private var exe = ""

    var body: some View {
        HStack(alignment: .firstTextBaseline) {
            VStack(alignment: .leading, spacing: 2) {
                HStack {
                    Text(game.name).bold()
                    if game.kind != .other {
                        Text(game.kind.rawValue)
                            .font(.caption)
                            .padding(.horizontal, 5)
                            .background(.tint.opacity(0.2), in: .capsule)
                    }
                    if game.configured {
                        Image(systemName: "checkmark.circle.fill").foregroundStyle(.green)
                    }
                }
                if game.kind != .other && !model.hasNeutron {   // neutron's D3D11 is always DXMT
                    if game.graphics?.lowercased() == "dxmt" {
                        Label("DXMT", systemImage: "checkmark.circle").font(.caption).foregroundStyle(.secondary)
                    } else {
                        Label("Graphics is \(game.graphics ?? "NotProton's default"): choose DXMT on its Compatibility page in NotProton",
                              systemImage: "exclamationmark.triangle")
                            .font(.caption)
                            .foregroundStyle(.orange)
                    }
                }
                if game.prefix == nil {
                    Text("Launch it once with \(model.toolName) first.").font(.caption).foregroundStyle(.secondary)
                } else if !game.exes.isEmpty {
                    Picker("Executable", selection: $exe) {
                        ForEach(game.exes, id: \.self) { Text($0).tag($0) }
                    }
                    .labelsHidden()
                    .frame(maxWidth: 320)
                }
            }
            Spacer()
            if game.configured {
                Button("Undo") { model.setUp(game, exe: exe, undo: true) }
            }
            Button(game.configured ? "Set Up Again" : "Set Up") { model.setUp(game, exe: exe, undo: false) }
                .disabled(game.prefix == nil || exe.isEmpty || model.installedVersion == nil)
            if model.hasNeutron && model.installedVersion != nil {
                Button("Play") { model.playOrSetUp(game, exe: exe) }
                    .buttonStyle(.borderedProminent)
            } else if game.configured {
                Button("Play") { model.play(game) }
                    .buttonStyle(.borderedProminent)
            }
        }
        .onAppear { if exe.isEmpty { exe = game.exes.first ?? "" } }
        Divider()
    }
}

@main
struct MacVapoRApp: App {
    @StateObject private var model = Model()

    var body: some Scene {
        WindowGroup("MacVapoR") {
            ContentView()
                .environmentObject(model)
                .frame(minWidth: 680, minHeight: 560)
        }
        .windowResizability(.contentMinSize)
    }
}
