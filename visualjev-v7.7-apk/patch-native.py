from pathlib import Path
import sys

root=Path(sys.argv[1])
p=root/"app/src/main/java/com/ryankeller/visualjev/MainActivity.kt"
s=p.read_text()
old="""                return try {
                    filePicker.launch(fileChooserParams?.createIntent())
                    true
                } catch (_: Throwable) {
                    fileCallback = null
                    false
                }"""
new="""                val chooserIntent = fileChooserParams?.createIntent()
                if (chooserIntent == null) {
                    fileCallback = null
                    return false
                }
                return try {
                    filePicker.launch(chooserIntent)
                    true
                } catch (_: Throwable) {
                    fileCallback = null
                    false
                }"""
if old not in s:
    raise SystemExit("MainActivity chooser patch target missing")
p.write_text(s.replace(old,new,1))
print("native patch applied")
