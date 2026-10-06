// Host-side verification helper for the probe firmware.
//
//   swift probe_check.swift list            list CoreMIDI endpoints and CoreAudio output devices
//   swift probe_check.swift select <name>   make the named CoreAudio device the default output
//   swift probe_check.swift midisend <name> send a note on/off pair to the named MIDI destination
//   swift probe_check.swift midirecv <name> <sec>  print MIDI bytes arriving from the named source

import CoreMIDI
import CoreAudio
import Foundation

func midiName(_ obj: MIDIObjectRef) -> String {
    var cf: Unmanaged<CFString>?
    guard MIDIObjectGetStringProperty(obj, kMIDIPropertyDisplayName, &cf) == noErr,
          let name = cf?.takeRetainedValue() else { return "<unnamed>" }
    return name as String
}

func audioDevices() -> [(AudioDeviceID, String, UInt32)] {
    var addr = AudioObjectPropertyAddress(mSelector: kAudioHardwarePropertyDevices,
                                          mScope: kAudioObjectPropertyScopeGlobal,
                                          mElement: kAudioObjectPropertyElementMain)
    var size: UInt32 = 0
    AudioObjectGetPropertyDataSize(AudioObjectID(kAudioObjectSystemObject), &addr, 0, nil, &size)
    var ids = [AudioDeviceID](repeating: 0, count: Int(size) / MemoryLayout<AudioDeviceID>.size)
    AudioObjectGetPropertyData(AudioObjectID(kAudioObjectSystemObject), &addr, 0, nil, &size, &ids)

    return ids.map { id in
        var nameAddr = AudioObjectPropertyAddress(mSelector: kAudioObjectPropertyName,
                                                  mScope: kAudioObjectPropertyScopeGlobal,
                                                  mElement: kAudioObjectPropertyElementMain)
        var cf: CFString = "" as CFString
        var cfSize = UInt32(MemoryLayout<CFString>.size)
        AudioObjectGetPropertyData(id, &nameAddr, 0, nil, &cfSize, &cf)

        var streamAddr = AudioObjectPropertyAddress(mSelector: kAudioDevicePropertyStreams,
                                                    mScope: kAudioDevicePropertyScopeOutput,
                                                    mElement: kAudioObjectPropertyElementMain)
        var streamSize: UInt32 = 0
        AudioObjectGetPropertyDataSize(id, &streamAddr, 0, nil, &streamSize)
        return (id, cf as String, streamSize / UInt32(MemoryLayout<AudioStreamID>.size))
    }
}

func sampleRate(_ id: AudioDeviceID) -> Double {
    var addr = AudioObjectPropertyAddress(mSelector: kAudioDevicePropertyNominalSampleRate,
                                          mScope: kAudioObjectPropertyScopeGlobal,
                                          mElement: kAudioObjectPropertyElementMain)
    var rate: Double = 0
    var size = UInt32(MemoryLayout<Double>.size)
    AudioObjectGetPropertyData(id, &addr, 0, nil, &size, &rate)
    return rate
}

func fourCC(_ id: UInt32) -> String {
    let bytes = [UInt8((id >> 24) & 0xff), UInt8((id >> 16) & 0xff),
                 UInt8((id >> 8) & 0xff), UInt8(id & 0xff)]
    let ascii = String(bytes: bytes.filter { (32..<127).contains($0) }, encoding: .ascii) ?? ""
    return ascii.count == 4 ? ascii : String(format: "0x%08x", id)
}

func uint32Property(_ id: AudioDeviceID, _ selector: AudioObjectPropertySelector,
                    scope: AudioObjectPropertyScope = kAudioObjectPropertyScopeGlobal) -> UInt32? {
    var addr = AudioObjectPropertyAddress(mSelector: selector, mScope: scope,
                                          mElement: kAudioObjectPropertyElementMain)
    var value: UInt32 = 0
    var size = UInt32(MemoryLayout<UInt32>.size)
    return AudioObjectGetPropertyData(id, &addr, 0, nil, &size, &value) == noErr ? value : nil
}

func stringProperty(_ id: AudioDeviceID, _ selector: AudioObjectPropertySelector,
                    scope: AudioObjectPropertyScope = kAudioObjectPropertyScopeGlobal) -> String? {
    var addr = AudioObjectPropertyAddress(mSelector: selector, mScope: scope,
                                          mElement: kAudioObjectPropertyElementMain)
    var cf: CFString = "" as CFString
    var size = UInt32(MemoryLayout<CFString>.size)
    guard AudioObjectGetPropertyData(id, &addr, 0, nil, &size, &cf) == noErr else { return nil }
    return cf as String
}

let args = CommandLine.arguments
let command = args.count > 1 ? args[1] : "list"

switch command {
case "list":
    print("CoreMIDI sources (device -> host):")
    for i in 0..<MIDIGetNumberOfSources() { print("  [\(i)] \(midiName(MIDIGetSource(i)))") }
    print("CoreMIDI destinations (host -> device):")
    for i in 0..<MIDIGetNumberOfDestinations() { print("  [\(i)] \(midiName(MIDIGetDestination(i)))") }

    var defAddr = AudioObjectPropertyAddress(mSelector: kAudioHardwarePropertyDefaultOutputDevice,
                                             mScope: kAudioObjectPropertyScopeGlobal,
                                             mElement: kAudioObjectPropertyElementMain)
    var current: AudioDeviceID = 0
    var curSize = UInt32(MemoryLayout<AudioDeviceID>.size)
    AudioObjectGetPropertyData(AudioObjectID(kAudioObjectSystemObject), &defAddr, 0, nil, &curSize, &current)

    print("CoreAudio output devices:")
    for (id, name, streams) in audioDevices() where streams > 0 {
        let transport = uint32Property(id, kAudioDevicePropertyTransportType).map(fourCC) ?? "?"
        let model = stringProperty(id, kAudioDevicePropertyModelUID) ?? "?"
        let src = uint32Property(id, kAudioDevicePropertyDataSource,
                                 scope: kAudioDevicePropertyScopeOutput).map(fourCC) ?? "-"
        let mark = id == current ? " <- default output" : ""
        print("  \(name) (id \(id), \(Int(sampleRate(id))) Hz, \(transport), src=\(src))\n      model \(model)\(mark)")
    }

case "select":
    let target = args[2]
    guard let match = audioDevices().first(where: { $0.2 > 0 && $0.1.contains(target) }) else {
        print("no output device matching \(target)"); exit(1)
    }
    var addr = AudioObjectPropertyAddress(mSelector: kAudioHardwarePropertyDefaultOutputDevice,
                                          mScope: kAudioObjectPropertyScopeGlobal,
                                          mElement: kAudioObjectPropertyElementMain)
    var id = match.0
    let status = AudioObjectSetPropertyData(AudioObjectID(kAudioObjectSystemObject), &addr, 0, nil,
                                            UInt32(MemoryLayout<AudioDeviceID>.size), &id)
    print(status == noErr ? "default output -> \(match.1)" : "failed: \(status)")
    exit(status == noErr ? 0 : 1)

case "midisend":
    let target = args[2]
    var client = MIDIClientRef()
    MIDIClientCreate("probe" as CFString, nil, nil, &client)
    var port = MIDIPortRef()
    MIDIOutputPortCreate(client, "out" as CFString, &port)

    var dest: MIDIEndpointRef?
    for i in 0..<MIDIGetNumberOfDestinations() {
        let d = MIDIGetDestination(i)
        if midiName(d).contains(target) { dest = d; break }
    }
    guard let destination = dest else { print("no MIDI destination matching \(target)"); exit(1) }

    for (status, note, velocity) in [(0x90, 64, 100), (0x80, 64, 0)] {
        var packetList = MIDIPacketList()
        let packet = MIDIPacketListInit(&packetList)
        _ = MIDIPacketListAdd(&packetList, 1024, packet, 0, 3,
                              [UInt8(status), UInt8(note), UInt8(velocity)])
        MIDISend(port, destination, &packetList)
        usleep(200_000)
    }
    print("sent note on/off to \(midiName(destination))")

case "midirecv":
    let target = args[2]
    let seconds = args.count > 3 ? Double(args[3]) ?? 3 : 3
    var client = MIDIClientRef()
    MIDIClientCreate("probe" as CFString, nil, nil, &client)
    var port = MIDIPortRef()
    MIDIInputPortCreateWithBlock(client, "in" as CFString, &port) { packetList, _ in
        var packet = packetList.pointee.packet
        for _ in 0..<packetList.pointee.numPackets {
            let bytes = withUnsafeBytes(of: packet.data) { Array($0.prefix(Int(packet.length))) }
            print("  rx: " + bytes.map { String(format: "%02X", $0) }.joined(separator: " "))
            packet = MIDIPacketNext(&packet).pointee
        }
    }

    var found = false
    for i in 0..<MIDIGetNumberOfSources() {
        let src = MIDIGetSource(i)
        if midiName(src).contains(target) { MIDIPortConnectSource(port, src, nil); found = true }
    }
    guard found else { print("no MIDI source matching \(target)"); exit(1) }

    print("listening to \(target) for \(seconds)s")
    RunLoop.current.run(until: Date().addingTimeInterval(seconds))

default:
    print("unknown command \(command)")
    exit(1)
}
