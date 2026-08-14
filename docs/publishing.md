# 打包与发布（npm / tarball / git）

`@robotic-harness/dsh-bundle` 是一个标准的 npm 包，可通过三种方式安装为 DSH 插件：

1. **发布到 npm**（`dsh plugin add @robotic-harness/dsh-bundle`）—— 对用户最友好；
2. **交付 tarball**（`dsh plugin add ./robotic-harness-dsh-bundle-0.1.0.tgz`）—— 不需要 npm 账号，适合内部/演示分发；
3. **git 直接安装**（`dsh plugin add github:owner/repo`）—— 最方便但需要 `prepare` 授权。

## 包内容

`package.json` 的 `files` 字段控制发布内容：

```text
lib/            TS 构建产物（prepare/prepack 时由 tsc 生成）
cordis.patch.yml
skills/         25 个 SKILL.md
worker/         robotic_harness_worker 副本（prepack 时由 scripts/sync-worker.mjs 同步）
fixtures/       URDF/SDF/rosbag 夹具
scenarios/      MuJoCo 场景
README.md
```

`prepack` / `prepublishOnly` 钩子保证：打包或发布前自动 **同步 worker 副本 + 重新构建**，
因此从 npm 或 tarball 安装的包始终自包含且与 `python/` 最新代码一致。

## 本地验证打包（推荐先做）

```sh
# 在仓库根目录
pnpm --filter @robotic-harness/dsh-bundle pack --pack-destination ./dist-tarball
tar -tf dist-tarball/robotic-harness-dsh-bundle-0.1.0.tgz   # 检查内容
```

用 tarball 安装到 profile 验证：

```sh
export DSH_HOME=/f/dsh/.dsh-home
dsh plugin --profile rh-demo add ./dist-tarball/robotic-harness-dsh-bundle-0.1.0.tgz
dsh --profile rh-demo --dump-config   # 应出现 # == @robotic-harness/dsh-bundle 层
```

> 注意：从 tarball/npm 安装后，工具的 `pythonPath` 仍需指向一个装有
> mujoco/numpy/opencv 的 Python 3.10 解释器（在 profile 的 cordis.patch.yml 中配置，
> 见根 README）。

## 发布到 npm（维护者操作）

```sh
# 0) 确保版本号正确（semver；v0.1.x 为 demo 阶段）
# 1) 登录 npm（需要 @robotic-harness scope 的发布权限，或用个人 scope 改名后发布）
npm login
# 2) 发布（prepublishOnly 会自动同步 worker + 构建）
pnpm --filter @robotic-harness/dsh-bundle publish --access public
# 3) 打 tag
git tag v0.1.0 && git push origin v0.1.0
```

发布前检查（对照根 README 的“发布前检查清单”）：

- [ ] `python run_tests.py` 全绿；
- [ ] `pnpm --filter @robotic-harness/dsh-bundle typecheck && build` 通过；
- [ ] `docs/screenshots/` 有最新截图，README 数值（工具数/测试数）与实际一致；
- [ ] CHANGELOG 或发布说明中列出破坏性变更（若有）。

## git 直接安装（用户路径，仅建议对可信源码使用）

```sh
dsh plugin --profile rh-demo add github:dingkaihu63/dsh-robotic-harness
```

- pnpm ≥ 10 默认拒绝运行 git 依赖的 `prepare` 脚本；第一次 `add` 会失败并提示把
  包键加入该 profile 的 `pnpm-workspace.yaml` `allowBuilds` 后重试。
- 请如实看待这项授权：**允许该包的代码在安装时于你的机器上执行**。
- 建议锁定 commit：`github:dingkaihu63/dsh-robotic-harness#<sha>`。

## 版本策略

- `0.1.x`：demo 阶段，API 可随时破坏，README 顶部有免责声明；
- 首个“可对外宣传”版本建议在真实用户反馈一轮后再打 `0.2.0`；
- 每次发布更新 `docs/roadmap.md` 与 README 中的版本徽章。
