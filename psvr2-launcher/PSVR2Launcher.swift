// PS VR2 Launcher: plays Steam VR games on a PS VR2 through the PS VR2 runtime (libpsvr2xr,
// built by build-psvr2.sh, which also builds this app around it). It shows whether the headset
// is connected, switches the OpenXR runtime between the PS VR2 and MacVapoR's Vision Pro one,
// lists the Steam games MacVapoR has set up, and starts one through Steam with Play. Setting a
// game up stays in MacVapoR, which carries the files that go into the game's prefix.

import AppKit
import CoreGraphics
import IOKit
import SwiftUI

enum Paths {
    static let home = FileManager.default.homeDirectoryForCurrentUser
    static let activeRuntime = home.appending(path: ".config/openxr/1/active_runtime.json")
    static let keptRuntime = home.appending(path: ".config/openxr/1/active_runtime.json.before-psvr2xr")
    static let visionProRuntime = home.appending(path: "Library/Application Support/MacVapoR/runtime/libalvrxr.dylib")
    static let steam = home.appending(path: "Library/Application Support/Steam")
    static let config = home.appending(path: "Library/Application Support/psvr2xr/config.json")
    static let log = home.appending(path: "Library/Logs/psvr2xr.log")

    // build-psvr2.sh puts the runtime and the self-test inside the app.
    static let bundled = Bundle.main.bundleURL.appending(path: "Contents/Library/psvr2xr")
    static let runtime = bundled.appending(path: "libpsvr2xr.dylib")
    static let selftest = bundled.appending(path: "psvr2xr-selftest")
}

// MARK: headset

enum Headset {
    /// The PS VR2's USB device, as the runtime opens it (src/psvr2/cpsvr2.c).
    static func onUSB() -> Bool {
        let match = IOServiceMatching("IOUSBHostDevice") as NSMutableDictionary
        match["idVendor"] = 0x054C
        match["idProduct"] = 0x0CDE
        let service = IOServiceGetMatchingService(kIOMainPortDefault, match as CFDictionary)
        defer { if service != 0 { IOObjectRelease(service) } }
        return service != 0
    }

    enum Display { case extended(hz: Double), mirrored, missing }

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
}

// MARK: games

struct Game: Identifiable {
    let id: String
    let name: String
    let kind: String      // "OpenXR" or "OpenVR"
    let configured: Bool  // MacVapoR has put wineopenxr into its prefix
}

/// The Steam libraries' VR games, found as MacVapoR's GameScanner finds them.
enum GameScanner {
    static func libraries() -> [URL] {
        var libs = [Paths.steam]
        let vdf = Paths.steam.appending(path: "steamapps/libraryfolders.vdf")
        if let text = try? String(contentsOf: vdf, encoding: .utf8) {
            for line in text.split(separator: "\n") where line.contains("\"path\"") {
                let parts = line.split(separator: "\"")
                if parts.count >= 4 {
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

    static func scan() -> [Game] {
        let fm = FileManager.default
        var games: [Game] = []
        for lib in libraries() {
            let apps = lib.appending(path: "steamapps")
            guard let items = try? fm.contentsOfDirectory(atPath: apps.path) else { continue }
            for item in items where item.hasPrefix("appmanifest_") && item.hasSuffix(".acf") {
                guard let acf = try? String(contentsOf: apps.appending(path: item), encoding: .utf8),
                      let id = value("appid", in: acf), let name = value("name", in: acf),
                      let installdir = value("installdir", in: acf) else { continue }
                if id == "228980" || id == "250820" || games.contains(where: { $0.id == id }) { continue }
                var kind = ""
                if let walker = fm.enumerator(at: apps.appending(path: "common/\(installdir)"), includingPropertiesForKeys: nil) {
                    for case let url as URL in walker {
                        if walker.level > 8 { walker.skipDescendants(); continue }
                        let file = url.lastPathComponent.lowercased()
                        if file == "openvr_api.dll" { kind = "OpenVR" }
                        if file == "openxr_loader.dll" && kind.isEmpty { kind = "OpenXR" }
                    }
                }
                guard !kind.isEmpty else { continue }
                let prefixes = [apps.appending(path: "compatdata/\(id)/pfx"),
                                Paths.steam.appending(path: "steamapps/compatdata/\(id)/pfx")]
                let configured = prefixes.contains {
                    fm.fileExists(atPath: $0.appending(path: "drive_c/Program Files (x86)/Steam/aarch64-windows/wineopenxr.dll").path)
                }
                games.append(Game(id: id, name: name, kind: kind, configured: configured))
            }
        }
        return games.sorted { $0.name.localizedCaseInsensitiveCompare($1.name) == .orderedAscending }
    }
}

// MARK: settings

/// The runtime's config.json (backend_psvr2.m reads it when a game starts).
struct Settings: Equatable {
    var renderScale = 0.75
    var eyeHeight = 1.6
    var lookaheadMs = 0.0
    var chromatic = true
    var reprojection = true
    var positional = false

    static func load() -> Settings {
        var s = Settings()
        guard let data = try? Data(contentsOf: Paths.config),
              let c = try? JSONSerialization.jsonObject(with: data) as? [String: Any] else { return s }
        s.renderScale = c["render_scale"] as? Double ?? s.renderScale
        s.eyeHeight = c["eye_height"] as? Double ?? s.eyeHeight
        s.lookaheadMs = c["lookahead_ms"] as? Double ?? s.lookaheadMs
        s.chromatic = c["chromatic"] as? Bool ?? s.chromatic
        s.reprojection = c["reprojection"] as? Bool ?? s.reprojection
        s.positional = c["positional"] as? Bool ?? s.positional
        return s
    }

    func save() {
        let c: [String: Any] = ["render_scale": renderScale, "eye_height": eyeHeight, "lookahead_ms": lookaheadMs,
                                "chromatic": chromatic, "reprojection": reprojection, "positional": positional]
        try? FileManager.default.createDirectory(at: Paths.config.deletingLastPathComponent(), withIntermediateDirectories: true)
        if let out = try? JSONSerialization.data(withJSONObject: c, options: [.prettyPrinted, .sortedKeys]) {
            try? out.write(to: Paths.config, options: .atomic)
        }
    }
}

// MARK: model

@MainActor
final class Model: ObservableObject {
    enum Runtime { case psvr2, visionPro, other(String), none }

    @Published var usb = false
    @Published var display = Headset.Display.missing
    @Published var runtime = Runtime.none
    @Published var games: [Game] = []
    @Published var settings = Settings.load()
    @Published var output = ""
    @Published var selftest: Process?

    var hasRuntime: Bool { FileManager.default.fileExists(atPath: Paths.runtime.path) }
    var hasVisionProRuntime: Bool { FileManager.default.fileExists(atPath: Paths.visionProRuntime.path) }
    private var timer: Timer?

    init() {
        refresh()
        // The headset comes and goes; its status is cheap to read.
        timer = Timer.scheduledTimer(withTimeInterval: 2, repeats: true) { [weak self] _ in
            Task { @MainActor in self?.refreshHeadset() }
        }
    }

    func refresh() {
        refreshHeadset()
        games = GameScanner.scan()
    }

    func refreshHeadset() {
        usb = Headset.onUSB()
        display = Headset.display()
        runtime = Self.activeRuntime()
    }

    static func activeRuntime() -> Runtime {
        guard let data = try? Data(contentsOf: Paths.activeRuntime),
              let json = try? JSONSerialization.jsonObject(with: data) as? [String: Any],
              let rt = json["runtime"] as? [String: Any], let lib = rt["library_path"] as? String else { return .none }
        if lib.hasSuffix("libpsvr2xr.dylib") { return .psvr2 }
        if lib.hasSuffix("libalvrxr.dylib") { return .visionPro }
        return .other(lib)
    }

    /// Points the OpenXR loader, and so every game MacVapoR set up, at a runtime.
    func useRuntime(psvr2: Bool) {
        let fm = FileManager.default
        do {
            try fm.createDirectory(at: Paths.activeRuntime.deletingLastPathComponent(), withIntermediateDirectories: true)
            // A runtime that is neither ours nor MacVapoR's is kept, as tools/psvr2-runtime.sh does.
            if case .other = Self.activeRuntime(), let old = try? Data(contentsOf: Paths.activeRuntime) {
                try? old.write(to: Paths.keptRuntime, options: .atomic)
            }
            let (name, lib) = psvr2 ? ("psvr2xr", Paths.runtime.path) : ("alvrxr", Paths.visionProRuntime.path)
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
            output = psvr2 ? "Games now show on the PS VR2." : "Games now show on the Vision Pro through ALVR."
        } catch {
            output = "Could not switch the runtime: \(error.localizedDescription)"
        }
        refreshHeadset()
    }

    /// Starts a game through Steam, which runs it with NotProton as its compatibility tool.
    func play(_ game: Game) {
        if case .psvr2 = runtime {} else { useRuntime(psvr2: true) }
        settings.save()
        var notes: [String] = []
        if !usb { notes.append("the headset is not on USB, so there will be no head tracking") }
        switch display {
        case .missing: notes.append("no PS VR2 display was found, so nothing will show in the headset")
        case .mirrored: notes.append("the PS VR2 display mirrors another one; set it to extend the desktop")
        case .extended: break
        }
        NSWorkspace.shared.open(URL(string: "steam://rungameid/\(game.id)")!)
        output = "Starting \(game.name) through Steam." + (notes.isEmpty ? "" : " Note: " + notes.joined(separator: "; ") + ".")
    }

    func startSelftest(slow: Bool) {
        guard selftest == nil else { return }
        let p = Process()
        p.executableURL = Paths.selftest
        p.arguments = slow ? ["--slow", "2"] : []
        p.terminationHandler = { _ in Task { @MainActor in self.selftest = nil } }
        do {
            try p.run()
            selftest = p
            output = "Self-test running: put the headset on. You should see a floor grid and four pillars."
        } catch {
            output = "Could not start the self-test: \(error.localizedDescription)"
        }
    }

    func stopSelftest() { selftest?.terminate() }
}

// MARK: views

struct ContentView: View {
    @EnvironmentObject var model: Model

    var body: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: 20) {
                headsetSection
                gamesSection
                settingsSection
                if !model.output.isEmpty {
                    Text(model.output)
                        .font(.callout)
                        .textSelection(.enabled)
                        .padding(8)
                        .frame(maxWidth: .infinity, alignment: .leading)
                        .background(.quaternary, in: .rect(cornerRadius: 6))
                }
            }
            .padding(20)
        }
        .toolbar {
            Button("Refresh", systemImage: "arrow.clockwise") { model.refresh() }
            Button("Runtime Log", systemImage: "doc.text") { NSWorkspace.shared.open(Paths.log) }
        }
        .onChange(of: model.settings) { _, s in s.save() }
    }

    private var headsetSection: some View {
        GroupBox("1. Headset") {
            VStack(alignment: .leading, spacing: 8) {
                status(model.usb, "On USB (head tracking)", "Not on USB: check the adapter's USB cable and that the headset is on")
                switch model.display {
                case .extended(let hz):
                    status(true, String(format: "Display connected at %.0f Hz%@", hz, hz > 100 ? " (90 Hz is easier on a MacBook Air)" : ""), "")
                case .mirrored:
                    status(false, "", "The PS VR2 display mirrors another one: set it to extend the desktop in Displays")
                case .missing:
                    status(false, "", "No PS VR2 display: check the DisplayPort cable and the adapter's power")
                }
                HStack {
                    switch model.runtime {
                    case .psvr2: Text("Games show on: PS VR2")
                    case .visionPro: Text("Games show on: Vision Pro (ALVR)")
                    case .other(let lib): Text("Games show on: another runtime (\(lib))")
                    case .none: Text("No OpenXR runtime is active.")
                    }
                    Spacer()
                    Button("Use PS VR2") { model.useRuntime(psvr2: true) }
                        .disabled(!model.hasRuntime)
                    Button("Use Vision Pro") { model.useRuntime(psvr2: false) }
                        .disabled(!model.hasVisionProRuntime)
                }
                HStack {
                    Text("Self-test: a test room in the headset, without a game.").foregroundStyle(.secondary)
                    Spacer()
                    if model.selftest == nil {
                        Button("Run") { model.startSelftest(slow: false) }
                        Button("Run at half rate") { model.startSelftest(slow: true) }
                    } else {
                        Button("Stop") { model.stopSelftest() }
                    }
                }
            }
            .frame(maxWidth: .infinity, alignment: .leading)
            .padding(4)
        }
    }

    private func status(_ ok: Bool, _ good: String, _ bad: String) -> some View {
        Label(ok ? good : bad, systemImage: ok ? "checkmark.circle" : "exclamationmark.triangle")
            .foregroundStyle(ok ? Color.secondary : Color.orange)
    }

    private var gamesSection: some View {
        GroupBox("2. Steam VR games") {
            VStack(alignment: .leading, spacing: 10) {
                Text("Set a game up in MacVapoR first. Play switches games to the PS VR2 and starts the game through Steam. There are no motion controllers yet, so use a gamepad.")
                    .font(.callout)
                    .foregroundStyle(.secondary)
                if model.games.isEmpty {
                    Text("No VR games found in your Steam libraries.")
                }
                ForEach(model.games) { game in
                    HStack {
                        Text(game.name).bold()
                        Text(game.kind)
                            .font(.caption)
                            .padding(.horizontal, 5)
                            .background(.tint.opacity(0.2), in: .capsule)
                        Spacer()
                        if game.configured {
                            Button("Play") { model.play(game) }
                                .disabled(!model.hasRuntime)
                        } else {
                            Text("Set it up in MacVapoR").font(.caption).foregroundStyle(.secondary)
                        }
                    }
                    Divider()
                }
            }
            .frame(maxWidth: .infinity, alignment: .leading)
            .padding(4)
        }
    }

    private var settingsSection: some View {
        GroupBox("3. Settings") {
            VStack(alignment: .leading, spacing: 8) {
                HStack {
                    Text("Render scale")
                    Slider(value: $model.settings.renderScale, in: 0.4...1.2, step: 0.05).frame(maxWidth: 260)
                    Text(String(format: "%.2f", model.settings.renderScale)).monospacedDigit()
                }
                Text("Lower it if games stutter.").font(.caption).foregroundStyle(.secondary)
                HStack {
                    Text("Eye height")
                    Slider(value: $model.settings.eyeHeight, in: 1.0...2.0, step: 0.05).frame(maxWidth: 260)
                    Text(String(format: "%.2f m", model.settings.eyeHeight)).monospacedDigit()
                }
                HStack {
                    Text("Extra prediction")
                    Slider(value: $model.settings.lookaheadMs, in: -10...20, step: 1).frame(maxWidth: 260)
                    Text(String(format: "%+.0f ms", model.settings.lookaheadMs)).monospacedDigit()
                }
                Toggle("Smooth head turns (reprojection)", isOn: $model.settings.reprojection)
                Toggle("Per-colour lens correction", isOn: $model.settings.chromatic)
                Toggle("Room-scale position (experimental)", isOn: $model.settings.positional)
                Text("Takes effect the next time a game starts.").font(.caption).foregroundStyle(.secondary)
            }
            .frame(maxWidth: .infinity, alignment: .leading)
            .padding(4)
        }
    }
}

@main
struct PSVR2LauncherApp: App {
    @StateObject private var model = Model()

    var body: some Scene {
        WindowGroup("PS VR2 Launcher") {
            ContentView()
                .environmentObject(model)
                .frame(minWidth: 620, minHeight: 520)
        }
        .windowResizability(.contentMinSize)
    }
}
