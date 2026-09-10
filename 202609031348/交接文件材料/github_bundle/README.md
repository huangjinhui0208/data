# GitHub 原始数据分卷

`raw/` 原始数据总量约 16.6 GiB，超过 GitHub 普通 Git 单文件限制。本目录保存其 gzip 压缩 tar 包的 Git LFS 分卷；每个分卷均小于 2 GB。

克隆仓库后，在本目录运行：

```powershell
powershell -ExecutionPolicy Bypass -File .\restore_raw_bundle.ps1
```

脚本会按清单顺序合并分卷、校验完整归档 SHA-256，并将 `raw/` 恢复到交接材料根目录。为防止覆盖现场数据，如果目标 `raw/` 已存在，脚本会直接退出。

`RAW_BUNDLE_MANIFEST.csv` 记录完整归档与每个分卷的字节数和 SHA-256。恢复后可再使用上一级 `index/raw_data_inventory.csv` 核验每个原始文件。

