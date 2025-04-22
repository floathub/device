const {
	updateModuleSha256,
	updateModuleCrc32,
	updateModulePrefix,
	updateModuleSuffix,
	compressModule,
	decompressModule,
	combineModules,
	splitCombinedModules
} = require('../../lib/moduleEncoding');

const HalModuleParser = require('../../lib/HalModuleParser');
const { Flags: ModuleFlags, MODULE_PREFIX_SIZE } = require('../../lib/ModuleInfo');
const { createFirmwareBinary } = require('../../lib/firmwareTestHelper');
const { config } = require ('../../lib/config');

const crc32 = require('buffer-crc32');
const { expect } = require('chai');

const crypto = require('crypto');
const path = require('path');
const fs = require('fs');

const TEST_BINARIES_PATH = path.resolve(path.join(__dirname, '../binaries'));

// Exclude some old and malformed firmware binaries from the test
const EXCLUDED_TEST_BINARIES = [
	'emptybin.bin',
	'RC2_bootloader.bin',
	'RC4_bootloader_pad_BM-09.bin',
	'RC4_bootloader_pad_BM-14.bin'
];

const TEST_BINARIES = fs.readdirSync(TEST_BINARIES_PATH)
		.filter(f => f.endsWith('.bin') && !EXCLUDED_TEST_BINARIES.includes(f))
		.map(f => path.join(TEST_BINARIES_PATH, f));

const SHA256_OFFSET = 38; // Relative to the end of the module data
const CRC32_OFFSET = 4; // ditto

function genRandomBuffer(size = 1024) {
	const buf = Buffer.alloc(size);
	for (let i = 0; i < buf.length; ++i) {
		buf[i] = Math.floor(Math.random() * 256);
	}
	return buf;
}

function genModuleBinary({ data, addVectorTable } = {}) {
	return createFirmwareBinary({
		buffer: data || genRandomBuffer(),
		productId: 0xabcd,
		productVersion: 0x1234,
		platformId: 10,
		addVectorTable
	});
}

async function parseModuleBinary(data) {
	const parser = new HalModuleParser();
	const info = await parser.parseBuffer({ fileBuffer: data });
	return info;
}

describe('moduleEncoding', () => {
	describe('updateModulePrefix()', () => {
		it('can update prefix fields selectively', async () => {
			const bin = genModuleBinary();
			let prefixInfo = {
				moduleStartAddy: 0x44332211,
				// moduleEndAddy: 0x88776655,
				reserved: 0x99,
				// moduleFlags: 0xaa,
				moduleVersion: 0xccbb,
				// platformID: 0xeedd,
				moduleFunction: 0xff,
				// moduleIndex: 0x11,
				depModuleFunction: 0x22,
				// depModuleIndex: 0x33,
				depModuleVersion: 0x5544,
				// dep2ModuleFunction: 0x66,
				dep2ModuleIndex: 0x77,
				// dep2ModuleVersion: 0x9988
			};
			bin.fill(0, 0, MODULE_PREFIX_SIZE);
			updateModulePrefix(bin, prefixInfo);
			let d = bin.slice(0, MODULE_PREFIX_SIZE);
			expect(d.toString('hex')).to.equal('11223344000000009900bbcc0000ff002200445500770000');
			prefixInfo = {
				// moduleStartAddy: 0x44332211,
				moduleEndAddy: 0x88776655,
				// reserved: 0x99,
				moduleFlags: 0xaa,
				// moduleVersion: 0xccbb,
				platformID: 0xeedd,
				// moduleFunction: 0xff,
				moduleIndex: 0x11,
				// depModuleFunction: 0x22,
				depModuleIndex: 0x33,
				// depModuleVersion: 0x5544,
				dep2ModuleFunction: 0x66,
				// dep2ModuleIndex: 0x77,
				dep2ModuleVersion: 0x9988
			};
			bin.fill(0, 0, MODULE_PREFIX_SIZE);
			updateModulePrefix(bin, prefixInfo);
			d = bin.slice(0, MODULE_PREFIX_SIZE);
			expect(d.toString('hex')).to.equal('000000005566778800aa0000ddee00110033000066008899');
		});

		it('correctly updates test module binaries', async () => {
			for (let file of TEST_BINARIES) {
				const origBin = fs.readFileSync(file);
				const { prefixInfo } = await parseModuleBinary(origBin);
				const bin = Buffer.from(origBin);
				bin.fill(0, prefixInfo.prefixOffset, prefixInfo.prefixOffset + MODULE_PREFIX_SIZE);
				updateModulePrefix(bin, prefixInfo);
				expect(bin.equals(origBin)).to.be.true;
			}
		});
	});

	describe('updateModuleSuffix()', () => {
		it('can update suffix fields selectively', async () => {
			const bin = genModuleBinary();
			let suffixInfo = {
				productId: 0x2211,
				// productVersion: 0x4433,
				reserved: 0x6655,
				// fwUniqueId: '778899aabbccddeeff112233445566778899aabbccddeeff1122334455667788',
				crcBlock: '99aabbcc',
				suffixSize: 40 // Mandatory
			};
			const offs = bin.length - suffixInfo.suffixSize - 4;
			const size = suffixInfo.suffixSize + 4; // Including CRC-32
			bin.fill(0, offs, offs + size);
			updateModuleSuffix(bin, suffixInfo);
			let d = bin.slice(offs, offs + size);
			expect(d.toString('hex')).to.equal('1122000055660000000000000000000000000000000000000000000000000000000000000000280099aabbcc');
			suffixInfo = {
				// productId: 0x2211,
				productVersion: 0x4433,
				// reserved: 0x6655,
				fwUniqueId: '778899aabbccddeeff112233445566778899aabbccddeeff1122334455667788',
				// crcBlock: '99aabbcc',
				suffixSize: 40
			};
			bin.fill(0, offs, offs + size);
			updateModuleSuffix(bin, suffixInfo);
			d = bin.slice(offs, offs + size);
			expect(d.toString('hex')).to.equal('000033440000778899aabbccddeeff112233445566778899aabbccddeeff1122334455667788280000000000');
		});

		it('correctly updates test module binaries', async () => {
			for (let file of TEST_BINARIES) {
				const origBin = fs.readFileSync(file);
				const { suffixInfo } = await parseModuleBinary(origBin);
				const bin = Buffer.from(origBin);
				const offs = bin.length - suffixInfo.suffixSize - 4;
				const size = suffixInfo.suffixSize + 4; // Including CRC-32
				bin.fill(0, offs, offs + size);
				updateModuleSuffix(bin, suffixInfo);
				expect(bin.equals(origBin)).to.be.true;
			}
		});
	});

	describe('compressModule() and decompressModule()', () => {
		it('compress and decompress a module binary', async () => {
			const bin = genModuleBinary({ data: Buffer.from('abababababababababcde'.repeat(10000)) });
			const comp = await compressModule(bin);
			expect(comp.length).to.be.lessThan(bin.length);
			const decomp = await decompressModule(comp);
			expect(decomp.equals(bin)).to.be.true;
		});

		it('set and clear COMPRESSED flag in the module\'s prefix info', async () => {
			const bin = genModuleBinary();
			let prefix = (await parseModuleBinary(bin)).prefixInfo;
			expect(prefix.moduleFlags & ModuleFlags.COMPRESSED).to.equal(0);
			const comp = await compressModule(bin);
			prefix = (await parseModuleBinary(comp)).prefixInfo;
			expect(prefix.moduleFlags & ModuleFlags.COMPRESSED).to.not.equal(0);
			const decomp = await decompressModule(comp);
			prefix = (await parseModuleBinary(decomp)).prefixInfo;
			expect(prefix.moduleFlags & ModuleFlags.COMPRESSED).to.equal(0);
		});

		it('encode the header of the compressed data correctly', async () => {
			const bin = genModuleBinary();
			const comp = await compressModule(bin, {
				zlib: {
					windowBits: 14
				}
			});
			const header = Buffer.alloc(8); // Expected header data
			let offs = 0;
			offs = header.writeUInt16LE(8, offs); // Header size
			offs = header.writeUInt8(0, offs); // Compression method
			offs = header.writeUInt8(14, offs); // Window size
			offs = header.writeUInt32LE(bin.length, offs); // Original size
			const h = comp.slice(MODULE_PREFIX_SIZE, MODULE_PREFIX_SIZE + header.length);
			expect(h.equals(header)).to.be.true;
		});

		it('produce a compressed module with valid checksums', async () => {
			const bin = genModuleBinary();
			const comp = await compressModule(bin);
			let hash = crypto.createHash('sha256');
			hash.update(comp.slice(0, comp.length - SHA256_OFFSET));
			hash = hash.digest();
			const crc = crc32(comp.slice(0, comp.length - CRC32_OFFSET));
			const info = await parseModuleBinary(comp);
			expect(info.suffixInfo.fwUniqueId).to.equal(hash.toString('hex'));
			expect(info.suffixInfo.crcBlock).to.equal(crc.toString('hex'));
			expect(info.crc.ok).to.be.ok;
		});

		it('can optionally preserve original module checksums', async () => {
			const bin = genModuleBinary();
			let offs = bin.length - SHA256_OFFSET;
			bin.fill(0x11, offs, offs + 32);
			offs = bin.length - CRC32_OFFSET;
			bin.fill(0x22, offs, offs + 4);
			const comp = await compressModule(bin, {
				updateSha256: false,
				updateCrc32: false
			});
			const { suffixInfo } = await parseModuleBinary(comp);
			expect(suffixInfo.fwUniqueId).to.equal('11'.repeat(32));
			expect(suffixInfo.crcBlock).to.equal('22'.repeat(4));
		});

		it('compress and decompress test module binaries successfully', async () => {
			for (let file of TEST_BINARIES) {
				const orig = fs.readFileSync(file);
				const comp = await compressModule(orig);
				expect(comp.length).to.be.lessThan(orig.length);
				const decomp = await decompressModule(comp);
				expect(decomp.equals(orig)).to.be.true;
			}
		});
	});

	describe('combineModules() and splitCombinedModules()', () => {
		it('combine and split module binaries', async () => {
			const bin1 = genModuleBinary();
			const bin2 = genModuleBinary();
			const comb = await combineModules([bin1, bin2]);
			expect(comb.length).to.equal(bin1.length + bin2.length);
			const uncomb = await splitCombinedModules(comb);
			expect(uncomb[0].equals(bin1)).to.be.true;
			expect(uncomb[1].equals(bin2)).to.be.true;
		});

		it('set and clear COMBINED flag on all modules except the last one', async () => {
			// Combine 3 modules
			const bin1 = genModuleBinary();
			let prefix = (await parseModuleBinary(bin1)).prefixInfo;
			expect(prefix.moduleFlags & ModuleFlags.COMBINED).to.equal(0);
			const bin2 = genModuleBinary();
			prefix = (await parseModuleBinary(bin2)).prefixInfo;
			expect(prefix.moduleFlags & ModuleFlags.COMBINED).to.equal(0);
			const bin3 = genModuleBinary();
			prefix = (await parseModuleBinary(bin3)).prefixInfo;
			expect(prefix.moduleFlags & ModuleFlags.COMBINED).to.equal(0);
			const comb = await combineModules([bin1, bin2, bin3]);
			// Check 1st module
			let buf = comb.slice(0, bin1.length);
			prefix = (await parseModuleBinary(buf)).prefixInfo;
			expect(prefix.moduleFlags & ModuleFlags.COMBINED).to.not.equal(0);
			// Check 2nd module
			buf = comb.slice(bin1.length, bin1.length + bin2.length);
			prefix = (await parseModuleBinary(buf)).prefixInfo;
			expect(prefix.moduleFlags & ModuleFlags.COMBINED).to.not.equal(0);
			// Check 3rd module
			buf = comb.slice(bin1.length + bin2.length, bin1.length + bin2.length + bin3.length);
			prefix = (await parseModuleBinary(buf)).prefixInfo;
			expect(prefix.moduleFlags & ModuleFlags.COMBINED).to.equal(0);
		});

		it('update checksums of combined modules', async () => {
			const bin1 = genModuleBinary();
			const bin2 = genModuleBinary();
			const comb = await combineModules([bin1, bin2]);
			// 1st module
			let buf = comb.slice(0, bin1.length);
			let hash = crypto.createHash('sha256');
			hash.update(buf.slice(0, buf.length - SHA256_OFFSET));
			hash = hash.digest();
			let crc = crc32(buf.slice(0, buf.length - CRC32_OFFSET));
			let info = await parseModuleBinary(buf);
			expect(info.suffixInfo.fwUniqueId).to.equal(hash.toString('hex'));
			expect(info.suffixInfo.crcBlock).to.equal(crc.toString('hex'));
			expect(info.crc.ok).to.be.ok;
			// 2nd module
			buf = comb.slice(bin1.length, bin1.length + bin2.length);
			hash = crypto.createHash('sha256');
			hash.update(buf.slice(0, buf.length - SHA256_OFFSET));
			hash = hash.digest();
			crc = crc32(buf.slice(0, buf.length - CRC32_OFFSET));
			info = await parseModuleBinary(buf);
			expect(info.suffixInfo.fwUniqueId).to.equal(hash.toString('hex'));
			expect(info.suffixInfo.crcBlock).to.equal(crc.toString('hex'));
			expect(info.crc.ok).to.be.ok;
		});

		it('can optionally preserve original module checksums', async () => {
			const bin1 = genModuleBinary();
			bin1.fill(0x11, bin1.length - SHA256_OFFSET, bin1.length - SHA256_OFFSET + 32);
			bin1.fill(0x22, bin1.length - CRC32_OFFSET, bin1.length - CRC32_OFFSET + 4);
			const bin2 = genModuleBinary();
			bin2.fill(0x33, bin2.length - SHA256_OFFSET, bin2.length - SHA256_OFFSET + 32);
			bin2.fill(0x44, bin2.length - CRC32_OFFSET, bin2.length - CRC32_OFFSET + 4);
			const comb = await combineModules([bin1, bin2], {
				updateSha256: false,
				updateCrc32: false
			});
			const uncomb = await splitCombinedModules(comb, {
				updateSha256: false,
				updateCrc32: false
			});
			let { suffixInfo } = await parseModuleBinary(uncomb[0]);
			expect(suffixInfo.fwUniqueId).to.equal('11'.repeat(32));
			expect(suffixInfo.crcBlock).to.equal('22'.repeat(4));
			({ suffixInfo } = await parseModuleBinary(uncomb[1]));
			expect(suffixInfo.fwUniqueId).to.equal('33'.repeat(32));
			expect(suffixInfo.crcBlock).to.equal('44'.repeat(4));
		});

		it('combine and split test module binaries successfully', async () => {
			const orig = [];
			const maxFiles = 10;
			let count = 0;
			for (let file of TEST_BINARIES) {
				const d = fs.readFileSync(file);
				orig.push(d);
				if (++count >= maxFiles) {
					break;
				}
			}
			const comb = await combineModules(orig);
			const uncomb = await splitCombinedModules(comb);
			expect(uncomb.length).to.equal(orig.length);
			for (let i = 0; i < uncomb.length; ++i) {
				expect(uncomb[i].equals(orig[i])).to.be.true;
			}
		});

		it('combine and split compressed module binaries successfully', async () => {
			const orig = [];
			const maxFiles = 5;
			let count = 0;
			for (let file of TEST_BINARIES) {
				const d = fs.readFileSync(file);
				orig.push(d);
				if (++count >= maxFiles) {
					break;
				}
			}
			const comp = [];
			for (let d of orig) {
				comp.push(await compressModule(d));
			}
			const comb = await combineModules(comp);
			const uncomb = await splitCombinedModules(comb);
			const uncomp = [];
			for (let d of uncomb) {
				uncomp.push(await decompressModule(d));
			}
			expect(uncomp.length).to.equal(orig.length);
			for (let i = 0; i < uncomp.length; ++i) {
				expect(uncomp[i].equals(orig[i])).to.be.true;
			}
		});
	});

	describe('updateModuleSha256()', () => {
		it('updates the SHA-256 checksum of a module', async () => {
			const bin = genModuleBinary();
			const offs = bin.length - SHA256_OFFSET;
			let hash = crypto.createHash('sha256');
			hash.update(bin.slice(0, offs));
			hash = hash.digest();
			bin.fill(0, offs, offs + 32);
			updateModuleSha256(bin);
			const { suffixInfo } = await parseModuleBinary(bin);
			expect(suffixInfo.fwUniqueId).to.equal(hash.toString('hex'));
		});
	});

	describe('updateModuleCrc32()', () => {
		it('updates the CRC-32 checksum of a module', async () => {
			const bin = genModuleBinary();
			const offs = bin.length - CRC32_OFFSET;
			const crc = crc32(bin.slice(0, offs));
			bin.fill(0, offs, offs + 4);
			updateModuleCrc32(bin);
			const { suffixInfo } = await parseModuleBinary(bin);
			expect(suffixInfo.crcBlock).to.equal(crc.toString('hex'));
		});

		describe('given that a custom CRC-32 function is provided globally', () => {
			let defaultCrc32 = null;

			before(() => {
				defaultCrc32 = config().crc32;
			});

			afterEach(() => {
				config({ crc32: defaultCrc32 });
			});

			it('uses that function for CRC-32 computations', async () => {
				const bin = genModuleBinary();
				bin.fill(0, bin.length - CRC32_OFFSET);
				const dummyCrc = Buffer.from([0xaa, 0xbb, 0xcc, 0xdd]);
				config({ crc32: buf => dummyCrc });
				updateModuleCrc32(bin);
				const { suffixInfo } = await parseModuleBinary(bin);
				expect(suffixInfo.crcBlock).to.equal(dummyCrc.toString('hex'));
			});
		});
	});
});
