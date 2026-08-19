# Проверка: серая сцена на «Среднем» качестве (Яндекс-модерация, п.1.15)

Дата: 2026-08-19. Машина: Windows 11, NVIDIA GeForce RTX 4080, Chrome (стабильный).
Билды собраны из этого репозитория: «старый» = HEAD `49d862f` как есть, «новый» = HEAD + фикс
`TIERS.medium` в `src/core/postfx.js` (см. «Фикс» ниже). Яндекс Браузера на машине нет —
все проверки в Chrome; ANGLE-бэкенды перебирались флагом `--use-angle=…` в изолированном
профиле, окно 2560×1400, DPR 1, тир форсировался `?quality=`.

## Итоговая таблица (билд × тир × бэкенд)

| Билд | Тир | d3d11 | gl (ANGLE поверх OpenGL) | swiftshader | d3d9* |
|---|---|---|---|---|---|
| старый (HEAD) | low | ок | ок | — | — |
| старый (HEAD) | **medium** | ок | **СЕРАЯ ПЕЛЕНА** | ок | ок (WARP) |
| старый (HEAD) | high | ок | ок | — | — |
| старый (HEAD) | ultra | ок | — | — | — |
| новый (фикс) | low | ок | ок | — | — |
| новый (фикс) | **medium** | ок | **ок** | ок | — |
| новый (фикс) | high | ок | ок | — | — |
| новый (фикс) | ultra | ок | ок | — | — |

\* `--use-angle=d3d9` в текущем Chrome недоступен: браузер молча падает в WARP
(`Microsoft Basic Render Driver`, программный D3D11) — там сцена целая. Это, кстати,
типичная конфигурация модерационных VM без GPU, и она НЕ ломается.

Циклы переключений Низ→Среднее→Высокое→Ультра→Среднее (панель настроек открыта, фон виден):
- старый билд, d3d11, UI-кликами: все состояния целы;
- новый билд, d3d11, UI-кликами: все состояния целы, финальный medium цел;
- новый билд, gl, программно (`quality:changed` с паузами 4 с и скриншотом на каждом шаге):
  все состояния целы, финальный medium цел. **Acceptance-критерий выполнен.**

## Воспроизведение и бисекция (старый билд, medium, --use-angle=gl)

GPU-строка в сломанной конфигурации:
`ANGLE (NVIDIA Corporation, NVIDIA GeForce RTX 4080/PCIe/SSE2, OpenGL 4.5.0)`.
В рабочей: `ANGLE (NVIDIA, NVIDIA GeForce RTX 4080 (0x00002704) Direct3D11 vs_5_0 ps_5_0, D3D11)`.

Ошибок в консоли НЕТ: `__DESCENT_ERRORS__ = []`, ни `THREE.WebGLProgram: Shader Error`,
ни `CONTEXT_LOST`. В шипнутом билде `checkShaderErrors=false`
(лог: `[postfx] precompiled 88 programs in 5171.3 ms (scene 1969.7, chain 3201.6,
parallel=true, checkShaderErrors=false)`), и драйверная линковка тоже проходит —
поломка полностью молчаливая.

Бисекция (каждый шаг — свежая загрузка `?quality=medium`):
1. `__DESCENT__.postfx = null` → **сцена появилась** — баг в postfx-цепочке.
2. `volumetricClouds = false` + `quality:changed` → пелена осталась.
3. `shadows = false` + `quality:changed` → пелена осталась.
4. По пассам через `postfx.setEnabled`: `hdr` off → пелена; `lens` off → пелена;
   **`ao` off → сцена появилась**. Виновник — N8AO.
5. Контрпробы: `aoSamples 12→16` (как high) не лечит; medium `msaa 0→2` (как high) —
   **лечит**. У high (msaa 2, AO on) и low (msaa 0, AO off) бага нет.

**Корневая причина:** комбинация «N8AOPostPass поверх не-мультисэмплового composer-буфера»
(`ao: true` + `msaa: 0`) существовала только в тире medium. На ANGLE-бэкенде GL этот
вариант молча даёт мусорный полноэкранный AO-композит (однородный серо-голубой градиент)
поверх сцены; d3d11/WARP/SwiftShader/Metal его переваривают. Отсюда и картина модерации:
low и high целы, medium — пелена; и невоспроизводимость на macOS.

## Фикс (в «новом» билде)

`src/core/postfx.js`, `TIERS.medium`: каждая генерирующая шейдерный вариант ручка
приведена к значению из high или low, включая комбинации:
- `msaa: 0 → 2` (== high; собственно лечащее изменение — AO теперь работает в той же
  MSAA-конфигурации, что и проверенный high);
- `aoSamples: 12 → 16` (== high);
- `mbSamples: 8 → 10` (== high);
- `smaa: MEDIUM → HIGH` (== high).

Стоимость: у medium появляется 2x MSAA сцены (по оценке в комментарии файла
+1.5–2.5 мс на 1080p) — принятая цена за то, что medium больше не компилирует ни одной
шейдерной конфигурации, не доказанной high/low.

**Остаточный риск:** `multisampling: Math.min(tier.msaa, renderer.capabilities.maxSamples || 0)` —
на устройстве, где `maxSamples` = 0, medium снова получит комбинацию «AO + msaa 0».
Если хочется брони и там, надо дополнительно гейтить AO на `multisampling > 0`
(или выключать AO на medium как в low).

## Артефакты

Скриншоты всех прогонов: scratchpad сессии, `shots/*.png`
(old-medium-{d3d11,gl,d3d9,swiftshader}, old-high-gl, old-medium-gl-{postfxnull,cloudsoff,shadowsoff,hdroff,lensoff,aooff},
new2-{low,medium,high,ultra}-gl, new2-medium-{d3d11,swiftshader}, new2-cycle-gl-step0..4).
CDP-раннер: `cdp-run.mjs` + `run-backend.ps1` там же.
