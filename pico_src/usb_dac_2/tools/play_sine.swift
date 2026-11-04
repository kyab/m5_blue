import AudioToolbox
import Darwin
import Foundation

// Play a 48 kHz stereo 1 kHz sine at -6 dBFS to the output named "Pico USB DAC".
// The name must contain that string. "Pico" alone is not enough.
// Both channels carry the same tone. The system default output is not used.
//
//   swiftc -O -o play_sine play_sine.swift
//   ./play_sine 1800

nonisolated(unsafe) var gStop: Int32 = 0
private let handleStop: @convention(c) (Int32) -> Void = { _ in
    gStop = 1
}

let seconds = CommandLine.arguments.count > 1 ? (Double(CommandLine.arguments[1]) ?? 0) : 0
let progressEvery = CommandLine.arguments.count > 2 ? (Double(CommandLine.arguments[2]) ?? 60) : 60

func fail(_ msg: String) -> Never {
    fputs("FAIL \(msg)\n", stderr)
    exit(1)
}

func check(_ label: String, _ err: OSStatus) {
    if err != noErr { fail("\(label) osstatus=\(err)") }
}

if seconds <= 0 {
    fail("usage: play_sine <seconds> [progress_sec]")
}

struct Dev {
    let id: AudioDeviceID
    let name: String
    let uid: String
    let inCh: Int
    let outCh: Int
    let rate: Float64
}

func cfStringProp(_ id: AudioObjectID, _ selector: AudioObjectPropertySelector) -> String {
    var addr = AudioObjectPropertyAddress(
        mSelector: selector,
        mScope: kAudioObjectPropertyScopeGlobal,
        mElement: kAudioObjectPropertyElementMain)
    var unmanaged: Unmanaged<CFString>?
    var size = UInt32(MemoryLayout<Unmanaged<CFString>?>.size)
    if AudioObjectGetPropertyData(id, &addr, 0, nil, &size, &unmanaged) != noErr { return "?" }
    return (unmanaged?.takeRetainedValue() as String?) ?? "?"
}

func channelCount(_ id: AudioDeviceID, _ scope: AudioObjectPropertyScope) -> Int {
    var addr = AudioObjectPropertyAddress(
        mSelector: kAudioDevicePropertyStreamConfiguration,
        mScope: scope,
        mElement: kAudioObjectPropertyElementMain)
    var size: UInt32 = 0
    if AudioObjectGetPropertyDataSize(id, &addr, 0, nil, &size) != noErr || size == 0 { return 0 }
    let raw = UnsafeMutableRawPointer.allocate(byteCount: Int(size), alignment: MemoryLayout<AudioBufferList>.alignment)
    defer { raw.deallocate() }
    if AudioObjectGetPropertyData(id, &addr, 0, nil, &size, raw) != noErr { return 0 }
    let abl = UnsafeMutableAudioBufferListPointer(UnsafeMutablePointer<AudioBufferList>(OpaquePointer(raw)))
    return abl.reduce(0) { $0 + Int($1.mNumberChannels) }
}

func nominalRate(_ id: AudioDeviceID) -> Float64 {
    var addr = AudioObjectPropertyAddress(
        mSelector: kAudioDevicePropertyNominalSampleRate,
        mScope: kAudioObjectPropertyScopeGlobal,
        mElement: kAudioObjectPropertyElementMain)
    var rate: Float64 = 0
    var size = UInt32(MemoryLayout<Float64>.size)
    if AudioObjectGetPropertyData(id, &addr, 0, nil, &size, &rate) != noErr { return 0 }
    return rate
}

func volumeDesc(_ id: AudioDeviceID, _ scope: AudioObjectPropertyScope) -> String {
    var parts: [String] = []
    for el: UInt32 in [kAudioObjectPropertyElementMain, 1, 2] {
        var addr = AudioObjectPropertyAddress(
            mSelector: kAudioDevicePropertyVolumeScalar,
            mScope: scope,
            mElement: el)
        guard AudioObjectHasProperty(id, &addr) else { continue }
        var vol: Float32 = 0
        var size = UInt32(MemoryLayout<Float32>.size)
        if AudioObjectGetPropertyData(id, &addr, 0, nil, &size, &vol) == noErr {
            parts.append(String(format: "el%u=%.3f", el, vol))
        }
    }
    return parts.isEmpty ? "none" : parts.joined(separator: " ")
}

func allDevices() -> [Dev] {
    var addr = AudioObjectPropertyAddress(
        mSelector: kAudioHardwarePropertyDevices,
        mScope: kAudioObjectPropertyScopeGlobal,
        mElement: kAudioObjectPropertyElementMain)
    var size: UInt32 = 0
    check("device list size", AudioObjectGetPropertyDataSize(AudioObjectID(kAudioObjectSystemObject), &addr, 0, nil, &size))
    let n = Int(size) / MemoryLayout<AudioDeviceID>.size
    var ids = [AudioDeviceID](repeating: 0, count: n)
    check("device list", AudioObjectGetPropertyData(AudioObjectID(kAudioObjectSystemObject), &addr, 0, nil, &size, &ids))
    return ids.map { id in
        Dev(
            id: id,
            name: cfStringProp(id, kAudioDevicePropertyDeviceNameCFString),
            uid: cfStringProp(id, kAudioDevicePropertyDeviceUID),
            inCh: channelCount(id, kAudioDevicePropertyScopeInput),
            outCh: channelCount(id, kAudioDevicePropertyScopeOutput),
            rate: nominalRate(id))
    }
}

func requireOne(_ label: String, _ matches: [Dev]) -> Dev {
    if matches.count != 1 {
        let desc = matches.map { "\($0.name) id=\($0.id) in=\($0.inCh) out=\($0.outCh)" }.joined(separator: " | ")
        fail("\(label) matches=\(matches.count) [\(desc)]")
    }
    return matches[0]
}

final class Tone {
    var phase: Double = 0
    let delta: Double
    let amp = Float(pow(10.0, -6.0 / 20.0))
    init(rate: Double) { delta = 2.0 * Double.pi * 1000.0 / rate }
}

let toneBox = Tone(rate: 48000)

let renderCB: AURenderCallback = { ref, _, _, _, frames, ioData in
    guard let ioData else { return noErr }
    let tone = Unmanaged<Tone>.fromOpaque(ref).takeUnretainedValue()
    let abl = UnsafeMutableAudioBufferListPointer(ioData)
    let n = Int(frames)
    let amp = tone.amp
    let delta = tone.delta
    var phase = tone.phase
    if abl.count >= 2 {
        let l = abl[0].mData!.assumingMemoryBound(to: Float.self)
        let r = abl[1].mData!.assumingMemoryBound(to: Float.self)
        for i in 0..<n {
            let s = amp * Float(sin(phase))
            l[i] = s
            r[i] = s
            phase += delta
            if phase >= 2 * Double.pi { phase -= 2 * Double.pi }
        }
    } else if let buf = abl.first, let data = buf.mData {
        let p = data.assumingMemoryBound(to: Float.self)
        if buf.mNumberChannels >= 2 {
            for i in 0..<n {
                let s = amp * Float(sin(phase))
                p[i * 2] = s
                p[i * 2 + 1] = s
                phase += delta
                if phase >= 2 * Double.pi { phase -= 2 * Double.pi }
            }
        } else {
            for i in 0..<n {
                p[i] = amp * Float(sin(phase))
                phase += delta
                if phase >= 2 * Double.pi { phase -= 2 * Double.pi }
            }
        }
    }
    tone.phase = phase
    return noErr
}

let devs = allDevices()
let play = requireOne(
    "Pico USB DAC output",
    devs.filter {
        $0.name.contains("Pico USB DAC") && !$0.name.contains("MIDI") && !$0.name.contains("Serial") && $0.outCh >= 2
    })
if !play.name.contains("Pico USB DAC") {
    fail("refusing to start: selected output is \(play.name)")
}
if abs(play.rate - 48000) > 1 {
    fail(String(format: "sample rate play=%.1f (wanted 48000, not changing it)", play.rate))
}
print(String(format: "play=\"%@\" id=%u uid=%@ rate=%.0f outVol[%@] amp=%.6f",
             play.name, play.id, play.uid, play.rate, volumeDesc(play.id, kAudioDevicePropertyScopeOutput), toneBox.amp))
fflush(stdout)

var desc = AudioComponentDescription(
    componentType: kAudioUnitType_Output,
    componentSubType: kAudioUnitSubType_HALOutput,
    componentManufacturer: kAudioUnitManufacturer_Apple,
    componentFlags: 0,
    componentFlagsMask: 0)
guard let comp = AudioComponentFindNext(nil, &desc) else { fail("HAL component missing") }
var playAU: AudioUnit?
check("AudioComponentInstanceNew", AudioComponentInstanceNew(comp, &playAU))
guard let playAU else { fail("nil AudioUnit") }

check("play enable out", {
    var v: UInt32 = 1
    return AudioUnitSetProperty(playAU, kAudioOutputUnitProperty_EnableIO, kAudioUnitScope_Output, 0, &v, 4)
}())
check("play disable in", {
    var v: UInt32 = 0
    return AudioUnitSetProperty(playAU, kAudioOutputUnitProperty_EnableIO, kAudioUnitScope_Input, 1, &v, 4)
}())
check("play set device", {
    var v = play.id
    return AudioUnitSetProperty(playAU, kAudioOutputUnitProperty_CurrentDevice, kAudioUnitScope_Global, 0, &v, UInt32(MemoryLayout<AudioDeviceID>.size))
}())
var fmt = AudioStreamBasicDescription(
    mSampleRate: play.rate,
    mFormatID: kAudioFormatLinearPCM,
    mFormatFlags: kAudioFormatFlagIsFloat | kAudioFormatFlagIsPacked,
    mBytesPerPacket: 8,
    mFramesPerPacket: 1,
    mBytesPerFrame: 8,
    mChannelsPerFrame: 2,
    mBitsPerChannel: 32,
    mReserved: 0)
check("play format", AudioUnitSetProperty(playAU, kAudioUnitProperty_StreamFormat, kAudioUnitScope_Input, 0, &fmt, UInt32(MemoryLayout<AudioStreamBasicDescription>.size)))
var renderProc = AURenderCallbackStruct(inputProc: renderCB, inputProcRefCon: Unmanaged.passUnretained(toneBox).toOpaque())
check("play callback", AudioUnitSetProperty(playAU, kAudioUnitProperty_SetRenderCallback, kAudioUnitScope_Input, 0, &renderProc, UInt32(MemoryLayout<AURenderCallbackStruct>.size)))
check("play init", AudioUnitInitialize(playAU))

signal(SIGTERM, handleStop)
signal(SIGINT, handleStop)
fputs("playing 1 kHz -6 dBFS on \(play.name) for \(seconds) s\n", stderr)
fflush(stderr)
check("play start", AudioOutputUnitStart(playAU))
let started = Date()
var nextMark = progressEvery
var stoppedEarly = false
while true {
    let elapsed = Date().timeIntervalSince(started)
    if gStop != 0 {
        stoppedEarly = true
        break
    }
    if elapsed >= seconds { break }
    if elapsed >= nextMark {
        fputs(String(format: "progress %.0f s\n", elapsed), stderr)
        fflush(stderr)
        nextMark += progressEvery
    }
    Thread.sleep(forTimeInterval: min(0.25, max(0.0, seconds - elapsed)))
}
check("play stop", AudioOutputUnitStop(playAU))
print(String(format: "played %.3f s stopped_early=%d", Date().timeIntervalSince(started), stoppedEarly ? 1 : 0))
fflush(stdout)
AudioUnitUninitialize(playAU)
AudioComponentInstanceDispose(playAU)
exit(stoppedEarly ? 2 : 0)
