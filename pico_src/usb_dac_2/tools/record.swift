import AudioToolbox
import Darwin
import Foundation

// Record the USB PnP input to a 48 kHz stereo int16 WAV.
// Samples are written while recording so a long capture is not held in RAM.
// Does not open an output device and does not change the system default devices.
//
//   swiftc -O -o record record.swift
//   ./record capture.wav 1800

nonisolated(unsafe) var gStop: Int32 = 0
private let handleStop: @convention(c) (Int32) -> Void = { _ in
    gStop = 1
}

let outPath = CommandLine.arguments.count > 1 ? CommandLine.arguments[1] : ""
let seconds = CommandLine.arguments.count > 2 ? (Double(CommandLine.arguments[2]) ?? 0) : 0
let progressEvery = CommandLine.arguments.count > 3 ? (Double(CommandLine.arguments[3]) ?? 60) : 60

func fail(_ msg: String) -> Never {
    fputs("FAIL \(msg)\n", stderr)
    exit(1)
}

func check(_ label: String, _ err: OSStatus) {
    if err != noErr { fail("\(label) osstatus=\(err)") }
}

if outPath.isEmpty || seconds <= 0 {
    fail("usage: record <wav> <seconds> [progress_sec]")
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

func pcmStereo(rate: Float64) -> AudioStreamBasicDescription {
    AudioStreamBasicDescription(
        mSampleRate: rate,
        mFormatID: kAudioFormatLinearPCM,
        mFormatFlags: kAudioFormatFlagIsFloat | kAudioFormatFlagIsPacked,
        mBytesPerPacket: 8,
        mFramesPerPacket: 1,
        mBytesPerFrame: 8,
        mChannelsPerFrame: 2,
        mBitsPerChannel: 32,
        mReserved: 0)
}

func makeHAL() -> AudioUnit {
    var desc = AudioComponentDescription(
        componentType: kAudioUnitType_Output,
        componentSubType: kAudioUnitSubType_HALOutput,
        componentManufacturer: kAudioUnitManufacturer_Apple,
        componentFlags: 0,
        componentFlagsMask: 0)
    guard let comp = AudioComponentFindNext(nil, &desc) else { fail("HAL component missing") }
    var au: AudioUnit?
    check("AudioComponentInstanceNew", AudioComponentInstanceNew(comp, &au))
    guard let au else { fail("nil AudioUnit") }
    return au
}

final class Capture {
    let capFrames = 8192
    let ringFrames = 48000 * 8
    let scratch: UnsafeMutablePointer<Float>
    let ring: UnsafeMutablePointer<Int16>
    var au: AudioUnit?
    var writePos = 0
    var readPos = 0
    var queued = 0
    var framesRecorded = 0
    var renderErrors = 0
    var overflows = 0
    var peakL: Float = 0
    var peakR: Float = 0
    var sumL: Double = 0
    var sumR: Double = 0
    let lock = NSLock()

    init() {
        scratch = .allocate(capacity: capFrames * 2)
        scratch.initialize(repeating: 0, count: capFrames * 2)
        ring = .allocate(capacity: ringFrames * 2)
        ring.initialize(repeating: 0, count: ringFrames * 2)
    }

    func enqueue(frames n: Int) {
        lock.lock()
        if queued + n > ringFrames {
            overflows += 1
            lock.unlock()
            return
        }
        var src = 0
        var left = n
        var pos = writePos
        while left > 0 {
            let take = min(left, ringFrames - pos)
            let dst = ring.advanced(by: pos * 2)
            for i in 0..<(take * 2) {
                var s = scratch[src + i]
                if s > 1 { s = 1 } else if s < -1 { s = -1 }
                let mag = abs(s)
                if i & 1 == 0 {
                    if mag > peakL { peakL = mag }
                    sumL += Double(s) * Double(s)
                } else {
                    if mag > peakR { peakR = mag }
                    sumR += Double(s) * Double(s)
                }
                dst[i] = Int16((Double(s) * 32767.0).rounded())
            }
            src += take * 2
            left -= take
            pos += take
            if pos == ringFrames { pos = 0 }
        }
        writePos = pos
        queued += n
        framesRecorded += n
        lock.unlock()
    }

    func dequeue(_ dst: UnsafeMutablePointer<Int16>, maxFrames: Int) -> Int {
        lock.lock()
        let n = min(queued, maxFrames)
        var left = n
        var pos = readPos
        var out = 0
        while left > 0 {
            let take = min(left, ringFrames - pos)
            let src = ring.advanced(by: pos * 2)
            dst.advanced(by: out).update(from: src, count: take * 2)
            out += take * 2
            left -= take
            pos += take
            if pos == ringFrames { pos = 0 }
        }
        readPos = pos
        queued -= n
        lock.unlock()
        return n
    }
}

let capture = Capture()

let inputCB: AURenderCallback = { ref, flags, ts, bus, frames, _ in
    let cap = Unmanaged<Capture>.fromOpaque(ref).takeUnretainedValue()
    guard let au = cap.au else { return kAudioUnitErr_Uninitialized }
    let n = Int(frames)
    if n <= 0 || n > cap.capFrames {
        cap.overflows += 1
        return noErr
    }
    cap.scratch.update(repeating: 0, count: n * 2)
    let buf = AudioBuffer(
        mNumberChannels: 2,
        mDataByteSize: UInt32(n * MemoryLayout<Float>.size * 2),
        mData: UnsafeMutableRawPointer(cap.scratch))
    var abl = AudioBufferList(mNumberBuffers: 1, mBuffers: buf)
    let err = AudioUnitRender(au, flags, ts, bus, frames, &abl)
    if err != noErr {
        cap.renderErrors += 1
        return err
    }
    cap.enqueue(frames: n)
    return noErr
}

let devs = allDevices()
let rec = requireOne("USB PnP input", devs.filter { $0.name.contains("USB PnP") && $0.inCh >= 2 })
if abs(rec.rate - 48000) > 1 {
    fail(String(format: "sample rate rec=%.1f (wanted 48000, not changing it)", rec.rate))
}
print(String(format: "rec=\"%@\" id=%u uid=%@ rate=%.0f inVol[%@]",
             rec.name, rec.id, rec.uid, rec.rate, volumeDesc(rec.id, kAudioDevicePropertyScopeInput)))
fflush(stdout)

let recAU = makeHAL()
check("rec disable out", {
    var v: UInt32 = 0
    return AudioUnitSetProperty(recAU, kAudioOutputUnitProperty_EnableIO, kAudioUnitScope_Output, 0, &v, 4)
}())
check("rec enable in", {
    var v: UInt32 = 1
    return AudioUnitSetProperty(recAU, kAudioOutputUnitProperty_EnableIO, kAudioUnitScope_Input, 1, &v, 4)
}())
check("rec set device", {
    var v = rec.id
    return AudioUnitSetProperty(recAU, kAudioOutputUnitProperty_CurrentDevice, kAudioUnitScope_Global, 0, &v, UInt32(MemoryLayout<AudioDeviceID>.size))
}())
check("rec format", {
    var fmt = pcmStereo(rate: rec.rate)
    return AudioUnitSetProperty(recAU, kAudioUnitProperty_StreamFormat, kAudioUnitScope_Output, 1, &fmt, UInt32(MemoryLayout<AudioStreamBasicDescription>.size))
}())
capture.au = recAU
var inputProc = AURenderCallbackStruct(inputProc: inputCB, inputProcRefCon: Unmanaged.passUnretained(capture).toOpaque())
check("rec callback", AudioUnitSetProperty(recAU, kAudioOutputUnitProperty_SetInputCallback, kAudioUnitScope_Global, 0, &inputProc, UInt32(MemoryLayout<AURenderCallbackStruct>.size)))
check("rec init", AudioUnitInitialize(recAU))

func wavHeader(dataBytes: UInt32) -> Data {
    var hdr = Data()
    func append32(_ v: UInt32) {
        var x = v.littleEndian
        withUnsafeBytes(of: &x) { hdr.append(contentsOf: $0) }
    }
    func append16(_ v: UInt16) {
        var x = v.littleEndian
        withUnsafeBytes(of: &x) { hdr.append(contentsOf: $0) }
    }
    hdr.append(contentsOf: "RIFF".utf8)
    append32(36 + dataBytes)
    hdr.append(contentsOf: "WAVE".utf8)
    hdr.append(contentsOf: "fmt ".utf8)
    append32(16)
    append16(1)
    append16(2)
    append32(48000)
    append32(48000 * 4)
    append16(4)
    append16(16)
    hdr.append(contentsOf: "data".utf8)
    append32(dataBytes)
    return hdr
}

guard let wav = fopen(outPath, "wb") else { fail("wav open \(outPath)") }
wavHeader(dataBytes: 0).withUnsafeBytes { raw in
    if fwrite(raw.baseAddress, 1, raw.count, wav) != raw.count { fail("wav header write") }
}
var drainBuf = [Int16](repeating: 0, count: 16384 * 2)
var writtenFrames = 0
func drain() {
    while true {
        let n = drainBuf.withUnsafeMutableBufferPointer { raw -> Int in
            capture.dequeue(raw.baseAddress!, maxFrames: 16384)
        }
        if n == 0 { break }
        let wrote = drainBuf.withUnsafeBytes { raw in
            fwrite(raw.baseAddress, 2, n * 2, wav)
        }
        if wrote != n * 2 { fail("wav pcm write") }
        writtenFrames += n
    }
}

signal(SIGTERM, handleStop)
signal(SIGINT, handleStop)
fputs("recording USB PnP for \(seconds) s -> \(outPath)\n", stderr)
fflush(stderr)
check("rec start", AudioOutputUnitStart(recAU))
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
    drain()
    if elapsed >= nextMark {
        capture.lock.lock()
        let got = capture.framesRecorded
        let errs = capture.renderErrors
        let overs = capture.overflows
        capture.lock.unlock()
        fputs(String(format: "progress %.0f s recorded=%.1f s renderErrors=%d overflows=%d\n",
                     elapsed, Double(got) / 48000.0, errs, overs), stderr)
        fflush(stderr)
        nextMark += progressEvery
    }
    Thread.sleep(forTimeInterval: min(0.25, max(0.0, seconds - elapsed)))
}
check("rec stop", AudioOutputUnitStop(recAU))
Thread.sleep(forTimeInterval: 0.05)
drain()
fflush(wav)
wavHeader(dataBytes: UInt32(writtenFrames * 4)).withUnsafeBytes { raw in
    if fseek(wav, 0, SEEK_SET) != 0 { fail("wav seek") }
    if fwrite(raw.baseAddress, 1, raw.count, wav) != raw.count { fail("wav header rewrite") }
}
fclose(wav)

capture.lock.lock()
let frames = capture.framesRecorded
let renderErrors = capture.renderErrors
let overflows = capture.overflows
let peakL = capture.peakL
let peakR = capture.peakR
let sumL = capture.sumL
let sumR = capture.sumR
capture.lock.unlock()

func db(_ lin: Float) -> Double {
    if lin <= 1e-8 { return -160 }
    return 20.0 * log10(Double(lin))
}
let rmsL = frames > 0 ? db(Float(sqrt(sumL / Double(frames)))) : -160
let rmsR = frames > 0 ? db(Float(sqrt(sumR / Double(frames)))) : -160
print(String(format: "wav=%@ frames=%d written=%d duration=%.3f s renderErrors=%d overflows=%d stopped_early=%d",
             outPath, frames, writtenFrames, Double(writtenFrames) / 48000.0, renderErrors, overflows, stoppedEarly ? 1 : 0))
print(String(format: "peak L=%.2f dBFS R=%.2f dBFS  rms L=%.2f dBFS R=%.2f dBFS", db(peakL), db(peakR), rmsL, rmsR))
fflush(stdout)
AudioUnitUninitialize(recAU)
AudioComponentInstanceDispose(recAU)
let complete = !stoppedEarly && writtenFrames >= Int(seconds * 48000.0) - 4800
exit(complete && renderErrors == 0 && overflows == 0 ? 0 : 2)
