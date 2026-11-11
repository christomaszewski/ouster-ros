# stb_image

`stb_image.h` is the single-header image decoder from <https://github.com/nothings/stb>,
copied unmodified. It reads the optional mask image (`mask_path`) in every format except
TIFF, which goes through libtiff; see `src/impl/mask_image.cpp`, the only file that includes it.

| | |
|---|---|
| Version | 2.30 |
| Upstream commit | `013ac3beddff3dbffafd5177e7972067cd2b5083` |
| SHA-256 | `594c2fe35d49488b4382dbfaec8f98366defca819d916ac95becf3e75f4200b3` |
| License | MIT or public domain, at the user's choice; the text is at the end of the header |

To update, replace the header with a newer upstream revision, update this table, and run the
`MaskImage` tests.
