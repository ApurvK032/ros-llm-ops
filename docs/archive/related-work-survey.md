# Related projects, papers, and models

> **Archived planning document.** Kept for history; it describes plans from before the system was built, not the current behavior. See the [design](../design.md) and [results](../results.md) for the current system.

Source review: 7 September 2026. Scope: a single robot receiving natural-language warehouse delivery requests, revising unfinished work, preserving cargo obligations, and executing through ROS 2/Nav2.

This review checks primary papers, author project pages, and repository documentation. It is not a reproduction study or a complete novelty search. Repository claims and reported paper results have not been independently measured here. “Not established” means the inspected evidence does not demonstrate a capability; it does not prove the capability is absent.

## Main finding

The broad idea already has substantial prior art, including classical planning plus LLM supervision, execution feedback, changing goals, preservation of ongoing actions, and instruction changes during manipulation. The portfolio project remains useful, but these broad capabilities should not be presented as new research contributions.

The most promising question to investigate further is the exact behavior when an instruction or proposal becomes stale across pickup, cancellation, and plan activation. Even timestamp-based stale-plan rejection is described in the closest paper. Investigate implementation and evaluation gaps rather than treating the check itself as new.

## Closest research systems

### 1. LLM-Assisted Plan Execution for Robots in Dynamic Environments — 2026

**Closest match found for the overall planning/execution architecture.** Uses PlanSys2, classical PDDL planning, LLM-assisted repair decisions, and continuity of compatible running actions. Its experimental setting includes picking and delivering objects between waypoints, changing connectivity, and changing goals. [Paper, published 15 April 2026](https://www.mdpi.com/2218-6581/15/4/80).

The public setup references PlanSys2's `rolling` branch and a fake Nav2 server with a modified action message. Its README describes Gemini, while the published experiment describes a Mistral model. Reproduction therefore needs a pinned code/model configuration and a compatibility audit before using our Jazzy stack. [Repository](https://github.com/IntelligentRoboticsLabs/llm-assisted-plan-execution).

**Use:** first architecture comparison and potential research baseline. Audit its proposal-to-activation behavior, carrying state, and cancellation races before designing a claimed extension. A standard Jazzy installation alone does not reproduce this project.

**Freshness detail:** Section 6, “Limitations,” describes rejecting a candidate when the current problem-state timestamp is newer than the state used to plan it. This is a described remedy; the inspected passage and repository documentation do not establish that it was implemented or experimentally evaluated. Treat it as direct conceptual prior art, with reproduction status unresolved. [Published article, Section 6](https://www.mdpi.com/2218-6581/15/4/80).

### 2. Agentic AI for Robot Control: Flexible but still Fragile — 2026

A language model invokes robot skills in an iterative planning/execution loop, queries semantic state, checks events, and accepts operator interventions during execution. The paper discusses instruction-following failures and prompt sensitivity, making it particularly relevant to our failure analysis. [Paper](https://arxiv.org/abs/2602.13081), [full text](https://arxiv.org/html/2602.13081v1).

The text provides tool/prompt details, and the [official project page](https://dfki-ni.github.io/AGENTS-MAKE-2026/index.html) links demonstrations, prompts, and [MobiPick Labs code](https://github.com/DFKI-NI/mobipick_labs). The underlying platforms use ROS 1; event handling is based on polling. Its reported validation is qualitative; do not turn it into a large benchmark claim. The linked robot stack does not by itself establish complete reproduction of the paper's agent orchestration.

**Use:** study intervention handling and event/state tool contracts. Determine how much consistency is enforced by code versus instructions given to the model.

### 3. CoMuRoS — 2025 preprint, updated 2026

Hierarchical task planning and execution for heterogeneous robot teams. It uses task/history/robot-state context; failures and user-intent changes trigger replanning. The architecture includes ROS 2 skills and model-generated executable Python. [Paper](https://arxiv.org/abs/2511.22354), [journal version](https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2026.1843313/full), [public repository](https://github.com/CoMuRoS/CoMuRoS).

**Use:** event relevance filtering, changed-intent scenarios, and recovery evaluation. Its multi-robot coordination and generated-code execution add scope beyond our one-robot, typed-tool design. Do not claim runtime user-intent replanning itself as our distinguishing feature.

### 4. SwitchVLA: Execution-Aware Task Switching for Vision-Language-Action Models — 2025

A learned policy conditions action generation on task language and execution/contact state, supporting forward, rollback, and advance behaviors. Its evaluations change instructions at early, middle, and late task phases. [Paper](https://arxiv.org/abs/2506.03574), [project and demonstrations](https://switchvla.github.io/).

**Use:** the most directly relevant model-level work for changes during execution, especially phase-relative evaluation. It targets manipulation policies rather than a ROS/Nav2 parcel mission manager. A public download for code/weights was not established from the inspected project page. There is no basis here for claiming it runs on our 12 GB GPU.

### 5. ROS-LLM — 2024 preprint; 2026 journal reference

Language-based robot task specification, ROS action/service execution, behavior sequences/trees/state machines, and feedback-driven reflection. [Paper](https://arxiv.org/abs/2406.19741), [original released code](https://github.com/huawei-noah/HEBO/tree/master/ROSLLM).

The original repository's README explicitly documents **ROS Noetic** and says ROS 2 support is planned. Treat it as an architectural reference, not a directly compatible Jazzy package. This statement is about that release, not every later project using a similar name. [Installation/support documentation](https://raw.githubusercontent.com/huawei-noah/HEBO/master/ROSLLM/README.md).

**Use:** supervisor/action boundaries, reusable behavior abstractions, and comparison framing.

### 6. VeriGraph — 2024 preprint; repository labels ICRA 2026

Builds scene-graph representations and checks action feasibility during iterative planning. The released repository supports online execution after verified batches through a ZeroMQ robot interface. [Paper](https://arxiv.org/abs/2411.10446), [repository](https://github.com/daniekpo/verigraph).

The release includes the execution client; the README says deployed perception/grasp components are not included. A compatible robot server and some data must be supplied. **Use:** independent action constraints, verified batches, and explicit state representations. ROS2/Nav2 warehouse integration and our cancellation protocol are not demonstrated by that README.

### 7. VerifyLLM — 2025

Translates task descriptions into temporal-logic specifications and uses LLM reasoning to assess/repair action sequences, including missing prerequisites, order mistakes, and redundant actions. [Paper](https://arxiv.org/abs/2507.05118), [project](https://verifyllm.github.io/), [repository](https://github.com/JohnSili/VerifyLLM).

**Use:** task-plan error categories and verification baselines. Its stated focus is pre-execution verification in household tasks. LLM reasoning guided by LTL does not itself give our application an atomic revision protocol or a guarantee of formal correctness.

### 8. Predictive vision-language monitoring for proactive safety in robot task execution — 2026

Monitors an ongoing action, requests cancellation, executes fallback behavior, and replans. The study uses Gazebo; it does not establish general semantic correctness merely from successful navigation. [Published paper](https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2026.1870024/full).

Its [ROS2 Feedback Planner repository](https://github.com/juandpenan/ros2_feedback_planner) explicitly targets Ubuntu 24.04, Jazzy and Harmonic, with navigation/manipulation demonstrations. Setup requires patches to several dependencies. **Use:** a close implementation reference for responsive monitoring and cancellation in our chosen simulator stack. It introduces visual monitoring beyond our text/logical-cargo MVP, and individual-order cancellation semantics still need investigation.

## Other papers worth reading

| Work | Relevant contribution | Implication for us |
| --- | --- | --- |
| [Inner Monologue, 2022](https://innermonologue.github.io/) | Uses execution and environmental feedback in language-driven embodied planning | Feedback-based replanning is established prior work |
| [SayCan, 2022](https://say-can.github.io/) | Combines language-based skill selection with estimated skill feasibility | Grounding a language model in executable skills predates this project |
| [SayPlan, CoRL 2023](https://sayplan.github.io/) | Scene-graph grounding, classical navigation planning, and simulated feedback to refine task plans | Combining an LLM with a conventional path planner is established |
| [As You Wish, 2026](https://arxiv.org/abs/2606.18519) | Natural-language mission planning with feedback and temporal-logic verification in agriculture | Mission verification must be distinguished from concurrent activation checks; [author resources](https://ucmercedrobotics.github.io/asUwish.html) |
| [Revisable by Design, 2026](https://arxiv.org/abs/2604.23283) | Streaming agent execution and revision costs classified by action reversibility | Read before claiming generic revision/rollback novelty; robotics transfer remains our inference |
| [Fresh Memory, Stale Plans, September 2026 preprint](https://arxiv.org/abs/2609.03340) | PlanFence checks the versions of action-relevant memory dependencies before execution | Direct prior art for freshness mechanisms outside robotics; inspect atomicity and declared-dependency assumptions before adapting it |
| [Priority-Driven Hierarchical Multi-Agent Systems with Fine-Tuned LLMs, August 2026](https://doi.org/10.3390/app16168250) | A single-robot supervisor, asynchronous tasks, behavior trees, and priority-driven interruption | Compare task termination and resource ownership; a cancellation request is not proof that robot motion has stopped; public implementation not verified |

## ROS frameworks and warehouse projects to inspect

| Project | What is documented | Likely role in our project |
| --- | --- | --- |
| [PlanSys2](https://github.com/PlanSys2/ros2_planning_system), [documentation](https://plansys2.github.io/) | ROS 2 PDDL planning infrastructure; release branches include Jazzy | Strong candidate if we adopt symbolic planning; compare effort against retaining our small greedy/grid core |
| [RAI — RobotecAI](https://github.com/RobotecAI/rai) | Agent tools and ROS 2 integration; explicitly documented [Jazzy/Humble setup](https://robotecai.github.io/rai/setup/install/) and [Nav2 goal, feedback, result, and cancellation tools](https://robotecai.github.io/rai/API_documentation/langchain_integration/ROS_2_tools/) | Strong candidate for the language-to-ROS adapter; parcel obligations and revision activation still need application logic |
| [ROSA — NASA JPL](https://github.com/nasa-jpl/rosa) | Natural-language inspection, diagnosis, and operation of ROS 1/2, with custom tools | Useful operations interface; a ready-made Jazzy parcel supervisor was not established |
| [EmbodiedAgents — Automatika Robotics/Inria](https://github.com/automatika-robotics/embodied-agents) | ROS 2 components for models, tools, events, and lifecycle/fallback management; [EMOS ecosystem documentation](https://emos.automatikarobotics.com/) | Broader orchestration option; its navigation ecosystem includes Kompass, so check Nav2 integration effort |
| [OmniPlan](https://github.com/mgonzs13/omni_plan) | ROS 2 planner/knowledge plugins, plan validation, and several execution mechanisms | Alternative planning infrastructure; a larger framework to learn and integrate |
| [Autonomous Warehouse AMR](https://github.com/Anastasios03git/autonomous-warehouse-amr) | README describes Jazzy/Harmonic/Nav2 warehouse simulation, YAML missions, station actions, retries, pause/resume/cancel and logs | Community reference for world/mission integration; language-driven revisions and cargo-preservation guarantees are not established |

The warehouse repository is visually and technically close to the intended simulator demo, but its documented mission cancellation is not evidence that cancelling an individual onboard order preserves obligations. Inspect behavior and source/asset licensing before reusing it. Keep the stock Nav2 environment as the first reproducible baseline.

## Models: which layer do they supply?

Our MVP needs a language model to interpret instructions and propose typed tool calls. Cargo state, action acknowledgements, and revision acceptance remain application responsibilities. A model checkpoint does not by itself supply this warehouse execution protocol.

| Model | What it supplies | Fit for this project |
| --- | --- | --- |
| [Qwen3.5-4B](https://huggingface.co/Qwen/Qwen3.5-4B), [9B](https://huggingface.co/Qwen/Qwen3.5-9B) | General language/vision models with documented agentic/function-calling use | Candidates for a later local supervisor comparison; no native ROS or parcel-state management. Quantized 4B is a reasonable first sizing experiment on the 4070, not a measured hardware recommendation |
| [SwitchVLA](https://switchvla.github.io/) | Execution-conditioned manipulation and task switching | Direct conceptual overlap with changed instructions; adopting it would introduce learned manipulation beyond the MVP |
| [OpenVLA](https://github.com/openvla/openvla) | A 7B image-and-language-to-action policy; released VLA models generate actions rather than conversational responses | Relevant if we later add learned grasping/control; no documented drop-in Nav2 supervisor |
| [Octo](https://octo-models.github.io/), [code](https://github.com/octo-models/octo) | Generalist action policies conditioned on language or goal images, with adaptable robot interfaces | Manipulation-policy reference; not a replacement for task-state management |
| [NVIDIA Isaac GR00T](https://github.com/NVIDIA/Isaac-GR00T) | Vision-language-action models and embodiment adaptation, including N1.7 in the inspected repository | Primarily learned robot control; the documented [hardware profile](https://github.com/NVIDIA/Isaac-GR00T/blob/main/getting_started/hardware_recommendation.md) calls for 16 GB+ inference and 40 GB+ fine-tuning, above the 4070's capacity |

No local model was installed or benchmarked in this review. Model memory depends on precision, context, runtime, and concurrent simulation; weights fitting into VRAM is insufficient evidence of usable latency. Keep the existing hosted-model-first MVP choice until the fake-client tool loop works. Local models remain an optional comparison after the demo.

## What this changes in our plan

1. Read the PlanSys2 repair paper and the DFKI agentic-control paper before committing to a research contribution.
2. Add a short integration investigation: inspect the repair repository's running-action preservation and current-state checks. Determine whether to reuse PlanSys2, adapt its ideas, or keep it as an external comparison.
3. Keep the portable state/event core small while evaluating that choice. Do not add both PlanSys2 and OmniPlan to the MVP.
4. Specify the differentiating scenarios precisely: cancellation before/after pickup acknowledgement; a delayed proposal after cargo changed; overlapping priority requests; a late success after cancel was requested; and a failed onboard drop.
5. Build a reproducible portfolio demo with attributed components. Claim a research contribution only if those scenarios reveal a gap and the proposed mechanism improves measured outcomes.

The practical reuse recommendation is provisional: **stock Nav2 for motion, our explicit cargo/request contracts, and a deliberate decision on PlanSys2 after examining the closest implementation.** RAI is the first language/ROS adapter to evaluate if a framework is useful. General agent or VLA frameworks should be adopted only if they remove more work than they add.
