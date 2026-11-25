//! The SD card's volume: block 0 must be an MBR whose partition 1 is a
//! FAT32 partition, type 0x0B (CHS) or 0x0C (LBA). Anything else (no 0x55AA, type 0x07 exFAT/NTFS,
//! 0xEE GPT, an empty entry, a FAT boot sector with no partition table) is the wrong format.

/// Offset of partition entry 1 in block 0.
const ENTRY1: usize = 0x1be;
/// Offset of the partition type in an entry.
const TYPE: usize = 4;
/// Offset of the first block (LBA, little-endian u32) in an entry.
const FIRST: usize = 8;
/// Offset of the block count (little-endian u32) in an entry.
const COUNT: usize = 12;
/// Offset of the boot signature 0x55 0xAA.
const SIGNATURE: usize = 510;

/// MBR partition 1 when it is a FAT32 partition inside the card.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Partition {
    /// 0x0B or 0x0C.
    pub kind: u8,
    /// The partition's first block.
    pub first_block: u32,
    /// The partition's length in blocks.
    pub blocks: u32,
}

/// Why block 0 does not describe a FAT32 volume.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WrongFormat {
    /// No 0x55AA at byte 510: no MBR at all.
    NoSignature,
    /// Partition 1's type is not 0x0B or 0x0C (0x07 exFAT/NTFS, 0xEE GPT, 0 empty, ...).
    Type(u8),
    /// Partition 1 starts at block 0 or has no blocks.
    Empty,
    /// Partition 1 reaches past the end of the card.
    BeyondCard,
}

/// Partition 1 of `block0` (the card's first 512 bytes) on a card of `capacity_blocks` blocks.
pub fn partition1(block0: &[u8; 512], capacity_blocks: u64) -> Result<Partition, WrongFormat> {
    let byte = |at: usize| block0.get(at).copied().unwrap_or(0);
    let word = |at: usize| u32::from_le_bytes([byte(at), byte(at + 1), byte(at + 2), byte(at + 3)]);
    if (byte(SIGNATURE), byte(SIGNATURE + 1)) != (0x55, 0xaa) {
        return Err(WrongFormat::NoSignature);
    }
    let kind = byte(ENTRY1 + TYPE);
    if kind != 0x0b && kind != 0x0c {
        return Err(WrongFormat::Type(kind));
    }
    let first_block = word(ENTRY1 + FIRST);
    let blocks = word(ENTRY1 + COUNT);
    if first_block == 0 || blocks == 0 {
        return Err(WrongFormat::Empty);
    }
    if u64::from(first_block) + u64::from(blocks) > capacity_blocks {
        return Err(WrongFormat::BeyondCard);
    }
    Ok(Partition {
        kind,
        first_block,
        blocks,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A real card: SanDisk 64 GB, 124 735 488 blocks.
    const CARD: u64 = 124_735_488;

    /// Block 0 with partition 1 = (`kind`, `first`, `blocks`) and the signature.
    fn mbr(kind: u8, first: u32, blocks: u32) -> [u8; 512] {
        let mut b = [0u8; 512];
        b[ENTRY1 + 1..ENTRY1 + 4].copy_from_slice(&[0xfe, 0xff, 0xff]);
        b[ENTRY1 + TYPE] = kind;
        b[ENTRY1 + 5..ENTRY1 + 8].copy_from_slice(&[0xfe, 0xff, 0xff]);
        b[ENTRY1 + FIRST..ENTRY1 + FIRST + 4].copy_from_slice(&first.to_le_bytes());
        b[ENTRY1 + COUNT..ENTRY1 + COUNT + 4].copy_from_slice(&blocks.to_le_bytes());
        b[SIGNATURE] = 0x55;
        b[SIGNATURE + 1] = 0xaa;
        b
    }

    #[test]
    fn the_bench_cards_mbr_is_a_fat32_volume() {
        let blocks = (CARD - 2048) as u32;
        assert_eq!(
            partition1(&mbr(0x0c, 2048, blocks), CARD),
            Ok(Partition {
                kind: 0x0c,
                first_block: 2048,
                blocks
            })
        );
        assert_eq!(
            partition1(&mbr(0x0b, 63, 1_000), CARD).map(|p| p.kind),
            Ok(0x0b)
        );
    }

    #[test]
    fn other_partition_types_are_the_wrong_format() {
        for kind in [0x00, 0x01, 0x04, 0x06, 0x07, 0x0e, 0x83, 0xee, 0xef] {
            assert_eq!(
                partition1(&mbr(kind, 2048, 1_000), CARD),
                Err(WrongFormat::Type(kind)),
                "{kind:#04x}"
            );
        }
    }

    #[test]
    fn no_signature_is_no_mbr() {
        let mut b = mbr(0x0c, 2048, 1_000);
        b[SIGNATURE + 1] = 0x00;
        assert_eq!(partition1(&b, CARD), Err(WrongFormat::NoSignature));
        assert_eq!(partition1(&[0; 512], CARD), Err(WrongFormat::NoSignature));
        assert_eq!(
            partition1(&[0xff; 512], CARD),
            Err(WrongFormat::NoSignature)
        );
    }

    #[test]
    fn a_fat_boot_sector_without_a_partition_table_is_the_wrong_format() {
        let mut b = [0u8; 512];
        b[..11].copy_from_slice(b"\xeb\x58\x90mkfs.fat");
        let message = b"This is not a bootable disk.  Please insert a bootable floppy";
        b[0x1a0..0x1a0 + message.len()].copy_from_slice(message);
        b[SIGNATURE] = 0x55;
        b[SIGNATURE + 1] = 0xaa;
        assert!(matches!(partition1(&b, CARD), Err(WrongFormat::Type(_))));
    }

    #[test]
    fn an_empty_or_oversized_partition_is_the_wrong_format() {
        assert_eq!(
            partition1(&mbr(0x0c, 0, 1_000), CARD),
            Err(WrongFormat::Empty)
        );
        assert_eq!(
            partition1(&mbr(0x0c, 2048, 0), CARD),
            Err(WrongFormat::Empty)
        );
        let fits = (CARD - 2048) as u32;
        assert_eq!(
            partition1(&mbr(0x0c, 2048, fits + 1), CARD),
            Err(WrongFormat::BeyondCard)
        );
        assert_eq!(
            partition1(&mbr(0x0c, u32::MAX, u32::MAX), CARD),
            Err(WrongFormat::BeyondCard)
        );
    }
}
