###  Case 1

> Process1: 1pub + x-sub
>
> Process2: 1sub

---

#### 1. 可选维度

- Process1执行器：`single-thread` or `multi-thread` 
- Process1 sub类型: `inter` or `intra`
- Process1 sub个数: `1` or `2` or `3`
- Process1 pub数据大小: `4b` or `4Kb` or `400Kb`

#### 2. 命名方式

| 序号  | 命名方式                  | 说明                                    | 状态 |
| :---: | :------------------------ | --------------------------------------- | ---- |
| **a** | **(a)_sig+1intra+4b.log** | **单线程+intra+1sub+Int32**, with trace | DONE |
| **b** | **(b)_mul+1intra+4b.log** | **多线程+intra+1sub+Int32**, with trace | DONE |
| **c** | **(c)_sig+2intra+4b.log** | **单线程+intra+2sub+Int32**, with trace | DONE |
| **d** | **(d)_mul+2intra+4b.log** | **多线程+intra+2sub+Int32**, with trace | DONE |
| **e** | **(e)_sig+3intra+4b.log** | **单线程+intra+3sub+Int32**             |      |
| **f** | **(f)_mul+3intra+4b.log** | **多线程+intra+3sub+Int32**             |      |
|   g   | (g)_sig+1intra+4Kb.log    |                                         |      |
|   h   | (h)_mul+1intra+4Kb.log    |                                         |      |
|   i   | (i)_sig+2intra+4Kb.log    |                                         |      |

