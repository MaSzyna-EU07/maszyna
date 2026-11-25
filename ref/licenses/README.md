# License texts of the Windows audio DLLs

The Windows build ships `OpenAL32.dll` (OpenAL Soft 1.25.2) and `libsndfile-1.dll` (libsndfile 1.2.2), and in
`licenses/` next to them the texts of those libraries and of what they have built in. The DLLs and OpenAL Soft's own
`COPYING` come from the release zips, which CI downloads; the texts below are kept here, so a build doesn't depend on
the servers they come from. CI copies them into `licenses/` and checks each one by the SHA-256 in
`.github/workflows/build_bin.yml` (step "Add the OpenAL Soft and libsndfile DLLs"). `.gitattributes` keeps them byte
for byte as upstream (`-text`), whatever the line ending settings of a checkout. What each text covers is in
`audio-libraries.txt`, which that step writes.

| File | Taken from | SHA-256 |
|------|------------|---------|
| `libsndfile-COPYING.txt` | https://raw.githubusercontent.com/libsndfile/libsndfile/1.2.2/COPYING | `ad01ea5cd2755f6048383c8d54c88459cd6fcb17757c5c8892f8c5ea060f6140` |
| `libsndfile-ALAC-LICENSE.txt` | https://raw.githubusercontent.com/libsndfile/libsndfile/1.2.2/src/ALAC/LICENSE | `232efbd83786e157286b0b96c68ebb673541b69b33036287bf1e5ec5067a49d9` |
| `libsndfile-GSM610-COPYRIGHT.txt` | https://raw.githubusercontent.com/libsndfile/libsndfile/1.2.2/src/GSM610/COPYRIGHT | `1bbe7119b1bcd1489e0f36cb95b8902f7f0ceec9a2f3a67394bc9f4709a4e0a7` |
| `libogg-COPYING.txt` | https://raw.githubusercontent.com/xiph/ogg/v1.3.5/COPYING | `d2ab5758336489da61c12cc5bb757da5339c4ae9001f9bb0562b4370249af814` |
| `libvorbis-COPYING.txt` | https://raw.githubusercontent.com/xiph/vorbis/v1.3.7/COPYING | `ec1815db59fcd302846df949d7424876cb2e2dc5ed1606c5fb0b36787b1cf43a` |
| `flac-COPYING.Xiph.txt` | https://raw.githubusercontent.com/xiph/flac/1.4.2/COPYING.Xiph | `6e58d3036f05e2a0d58af854c12a505ddbe75011d71a598a2170987f3b15408d` |
| `opus-COPYING.txt` | https://raw.githubusercontent.com/xiph/opus/v1.4/COPYING | `8338ce8d922bb4416ce3dd1e5680173332435e3f0755007ac7801ccd674fe682` |
| `mpg123-COPYING.txt` | https://scm.orgis.org/view/mpg123/tags/1.31.3/COPYING?view=co | `c22482728a634a8dfdb4ff72a96d4c1ed64cd8f3e79335c401751ac591609366` |
| `lame-COPYING.txt` | https://sources.debian.org/data/main/l/lame/3.100-6/COPYING (the same file as in LAME 3.100) | `bfe4a52dc4645385f356a8e83cc54216a293e3b6f1cb4f79f5fc0277abf937fd` |
| `openal-soft-pffft-LICENSE.txt` | https://raw.githubusercontent.com/kcat/openal-soft/1.25.2/LICENSE-pffft | `64056328b3e7bc104e24ef96accc1a7abead156f0c02d1b87c97e3f3a28030de` |
| `openal-soft-gsl-LICENSE.txt` | https://raw.githubusercontent.com/kcat/openal-soft/1.25.2/gsl/LICENSE | `4c21dc82a1c7186ea754fd59d504c00947953a3401fe8e24eaa44109ef8e2555` |
| `openal-soft-BSD-3Clause.txt` | https://raw.githubusercontent.com/kcat/openal-soft/1.25.2/BSD-3Clause | `b5a08cf98996dd83aa906a72c163600f6cb17a65ce546a553815a1267abd43ff` |
| `openal-soft-winpthreads-COPYING.txt` | https://raw.githubusercontent.com/mingw-w64/mingw-w64/v13.0.0/mingw-w64-libraries/winpthreads/COPYING | `63263614cdd29f2f93cba85e992f041b31f9fc7b4033692f31269489a8a1b177` |
| `openal-soft-mingw-w64-runtime.txt` | https://raw.githubusercontent.com/mingw-w64/mingw-w64/v13.0.0/COPYING.MinGW-w64-runtime/COPYING.MinGW-w64-runtime.txt | `e9b2dc02451ea29092a1f25fa0f3c07207ed421f1807dffb0c4e6dce69dee7bd` |

When a DLL is updated to another version, its texts here are replaced by those of the new version's source tree, and
their SHA-256 in the workflow with them.
