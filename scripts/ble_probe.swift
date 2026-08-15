// ble_probe.swift — CoreBluetooth probe for the ESP32_OSM_NAV "NAV-OSM" board.
// Usage:
//   swiftc ble_probe.swift -o ble_probe
//   ./ble_probe hello   # connect, read hello banner
//   ./ble_probe route   # connect + write a sample <route> packet
//   ./ble_probe pos     # connect + write a <pos> packet
//   ./ble_probe nav     # connect + write a <nav> packet
// Binary modes (new protocol): routeb posb navb etab clockb
import CoreBluetooth
import Foundation

let SRV = CBUUID(string: "5a7e1000-2b2f-4f66-9f9a-5c0f8e1a2b3c")
let CHR = CBUUID(string: "5a7e1001-2b2f-4f66-9f9a-5c0f8e1a2b3c")

class Probe: NSObject, CBCentralManagerDelegate, CBPeripheralDelegate {
    var cm: CBCentralManager!
    var chr: CBCharacteristic?
    var target: CBPeripheral?
    var done = false
    var pendingChunk: (() -> Void)? = nil   // next chunk sender, fired on write ack
    let mode = CommandLine.arguments.count > 1 ? CommandLine.arguments[1] : "hello"

    func start() { cm = CBCentralManager(delegate: self, queue: nil) }

    func centralManagerDidUpdateState(_ c: CBCentralManager) {
        if c.state == .poweredOn {
            print("[probe] scanning for service \(SRV.uuidString)...")
            c.scanForPeripherals(withServices: [SRV], options: [CBCentralManagerScanOptionAllowDuplicatesKey: false])
        } else {
            print("[probe] BT state rawValue=\(c.state.rawValue) (5=poweredOn)")
            exit(2)
        }
    }

    func centralManager(_ c: CBCentralManager, didDiscover p: CBPeripheral,
                        advertisementData: [String: Any], rssi RSSI: NSNumber) {
        let name = p.name ?? "(nil)"
        let svc = (advertisementData[CBAdvertisementDataServiceUUIDsKey] as? [CBUUID])?.map { $0.uuidString } ?? []
        print("[probe] FOUND \(name) rssi=\(RSSI) advServices=\(svc)")
        target = p
        c.stopScan()
        c.connect(p, options: nil)
    }

    func centralManager(_ c: CBCentralManager, didConnect p: CBPeripheral) {
        print("[probe] connected to \(p.name ?? "?")")
        p.delegate = self
        p.discoverServices([SRV])
    }

    func centralManager(_ c: CBCentralManager, didFailToConnect p: CBPeripheral, error: Error?) {
        print("[probe] connect FAILED: \(String(describing: error))")
        exit(1)
    }

    func centralManager(_ c: CBCentralManager, didDisconnectPeripheral p: CBPeripheral, error: Error?) {
        print("[probe] disconnected: \(String(describing: error))")
    }

    func peripheral(_ p: CBPeripheral, didDiscoverServices error: Error?) {
        if let e = error { print("[probe] svc error \(e)"); exit(1) }
        guard let svc = p.services?.first(where: { $0.uuid == SRV }) else {
            print("[probe] service not found; services=\(p.services?.map{$0.uuid.uuidString} ?? [])")
            exit(1)
        }
        print("[probe] service \(svc.uuid.uuidString) OK")
        p.discoverCharacteristics([CHR], for: svc)
    }

    func peripheral(_ p: CBPeripheral, didDiscoverCharacteristicsFor service: CBService, error: Error?) {
        guard let ch = service.characteristics?.first(where: { $0.uuid == CHR }) else {
            print("[probe] char not found; chars=\(service.characteristics?.map{$0.uuid.uuidString} ?? [])")
            exit(1)
        }
        chr = ch
        print("[probe] char \(ch.uuid.uuidString) props=\(ch.properties.rawValue) "
              + "(1=Broadcast 2=Read 4=WriteNR 8=Write 0x10=Notify 0x20=Indicate)")
        p.readValue(for: ch)
    }

    func peripheral(_ p: CBPeripheral, didUpdateValueFor ch: CBCharacteristic, error: Error?) {
        if let d = ch.value, let s = String(data: d, encoding: .utf8) {
            print("[probe] READ value: \"\(s)\"")
        } else {
            print("[probe] READ (empty or err \(String(describing: error)))")
        }
        runMode(p)
    }

    func peripheral(_ p: CBPeripheral, didWriteValueFor ch: CBCharacteristic, error: Error?) {
        print("[probe] write done err=\(String(describing: error))")
        if let c = pendingChunk { c() }   // fire the next chunk (if any)
    }

    func runMode(_ p: CBPeripheral) {
        var data: Data? = nil
        var chunked = false
        func i32(_ v: Int32) -> Data {
            let u = UInt32(bitPattern: v)
            return Data([UInt8(u & 0xff), UInt8((u >> 8) & 0xff),
                         UInt8((u >> 16) & 0xff), UInt8((u >> 24) & 0xff)])
        }
        func i16(_ v: Int16) -> Data {
            let u = UInt16(bitPattern: v)
            return Data([UInt8(u & 0xff), UInt8(u >> 8)])
        }
        func frame(_ type: UInt8, _ body: Data) -> Data {
            var d = Data([0xAA, 0x55, type])
            let n = body.count
            d.append(UInt8(n & 0xff)); d.append(UInt8((n >> 8) & 0xff))
            d.append(body)
            return d
        }
        switch mode {
        // ---- legacy XML modes (still supported by the board as a fallback) ----
        case "route":
            var b = "<route z=\"15\">"
            for i in 0..<8 {
                b += String(format: "<p lat=\"%.6f\" lon=\"%.6f\"/>", 10.7718 + Double(i) * 0.0005, 106.6982)
            }
            b += "</route>\u{0}"
            data = Data(b.utf8)
        case "pos":
            data = Data("<pos lat=\"10.77220\" lon=\"106.69910\" spd=\"34\" hdg=\"312\"></pos>\u{0}".utf8)
        case "nav":
            data = Data("<nav d=\"85\" m=\"left\" s=\"Lê Lợi\"></nav>\u{0}".utf8)
        case "eta":
            data = Data("<eta h=\"14\" m=\"32\" a=\"Ben Thanh\"></eta>\u{0}".utf8)
        case "clock":
            data = Data("<clock h=\"14\" m=\"30\"></clock>\u{0}".utf8)
        // ---- binary protocol modes ----
        case "routeb":
            var b = Data([15])                                   // zoom
            let cnt: UInt16 = 8
            b.append(UInt8(cnt & 0xff)); b.append(UInt8(cnt >> 8))
            b.append(i32(Int32(10.7718 * 1e7)))                  // lat0 x1e7
            b.append(i32(Int32(106.6982 * 1e7)))                 // lon0 x1e7
            for i in 1..<8 {                                     // deltas x1e5
                b.append(i16(Int16(50 * i)))
                b.append(i16(0))
            }
            data = frame(0x01, b)
        case "routec":   // same route as routeb but sent in <=20-byte chunks
            var b = Data([15])                                   // zoom
            let cnt: UInt16 = 8
            b.append(UInt8(cnt & 0xff)); b.append(UInt8(cnt >> 8))
            b.append(i32(Int32(10.7718 * 1e7)))                  // lat0 x1e7
            b.append(i32(Int32(106.6982 * 1e7)))                 // lon0 x1e7
            for i in 1..<8 {                                     // deltas x1e5
                b.append(i16(Int16(50 * i)))
                b.append(i16(0))
            }
            data = frame(0x01, b)
            chunked = true
        case "posb":
            var b = Data()
            b.append(i32(Int32(10.77220 * 1e7)))
            b.append(i32(Int32(106.69910 * 1e7)))
            b.append(34)                                          // spd
            b.append(0x38); b.append(0x01)                        // hdg 312 (LE u16)
            b.append(50)                                          // sl
            data = frame(0x02, b)
        case "navb":
            let s = Array("Lê Lợi".utf8)
            var b = Data([85, 0])                                 // dist 85 (LE u16)
            b.append(1)                                           // modId: left
            b.append(UInt8(s.count))
            b.append(contentsOf: s)
            data = frame(0x03, b)
        case "etab":
            let s = Array("Bến Thành".utf8)
            var b = Data([14, 32])                                // h, m
            b.append(UInt8(s.count))
            b.append(contentsOf: s)
            data = frame(0x04, b)
        case "clockb":
            data = frame(0x05, Data([14, 30]))
        default:
            break
        }
        if let d = data, let ch = chr {
            print("[probe] WRITE \(mode) (\(d.count) bytes\(chunked ? ", chunked" : ""))")
            if chunked {
                // send in <=20-byte chunks, each a single LL packet (the path
                // that never crashes this board's fragile controller)
                let step = 20
                var off = 0
                func sendChunk() {
                    guard off < d.count else {
                        print("[probe] chunks done")
                        DispatchQueue.main.asyncAfter(deadline: .now() + 1.5) {
                            print("[probe] done")
                            exit(0)
                        }
                        return
                    }
                    let n = min(step, d.count - off)
                    let sub = d.subdata(in: off..<(off + n))
                    off += n
                    p.writeValue(sub, for: ch, type: .withResponse)
                    // wait for the write callback before the next chunk
                    pendingChunk = { sendChunk() }
                }
                sendChunk()
            } else {
                p.writeValue(d, for: ch, type: .withResponse)
                DispatchQueue.main.asyncAfter(deadline: .now() + 1.5) {
                    print("[probe] done")
                    exit(0)
                }
            }
        } else {
            DispatchQueue.main.asyncAfter(deadline: .now() + 1.5) {
                print("[probe] done")
                exit(0)
            }
        }
    }
}

let probe = Probe()
probe.start()
RunLoop.main.run()
