# Preface

---

## Background of This Book

Designing autonomous driving algorithms is not finished by simply building a high-accuracy perception model. Vehicles drive on real roads and must deal with pedestrians, bicycles, oncoming vehicles, crossing vehicles, parked vehicles, construction work, rain, nighttime, missing lane markings, and regional traffic rules. If the system is considered as a product, real-time performance, power consumption, vehicle-specific steering characteristics, safety monitoring, logs, validation, regulations, and ODD management must all be satisfied at the same time.

This book systematizes an autonomous driving architecture that fuses Vision, LiDAR, and Radar in BEV, inputs external language instructions and internal scene summaries into the Planner, and outputs multiple candidate trajectories and confidence scores. It is written not only as a research design, but also with real-vehicle application and productization in mind.

---

## Position of This Book

Since 2023, autonomous driving research has been moving broadly in two directions.

One direction is "sensor fusion and End-to-End learning," as represented by BEVFormer, BEVFusion, and UniAD. Multiple sensors are fused into a common BEV space, and perception, prediction, and planning are handled in one learning pipeline. Diverse trajectory generation methods such as DiffusionDrive and SparseDrive have also appeared, increasing the maturity of the research field.

The other direction is "integration with language and reasoning," as represented by DriveLM and VLM-AD. Attempts to incorporate large language models and vision-language models into autonomous driving scene understanding and behavior planning are increasing rapidly.

However, many of these studies remain evaluated only by "nuScenes metrics" or "closed-loop simulators." To actually mount such systems in vehicles carrying people, one must consider computation, power, regulations, safety cases, ODD management, logs, sensor redundancy, fallback, verifiability, and OTA updates.

This book aims to bridge that gap between research and productization.

---

## Central Design Philosophy

This book has four central design principles.

**Use human trajectories, not lane centers, as the teacher.**  
The future trajectories actually driven by humans are used as the learning target for every processing cycle. This allows the system to learn implicit knowledge from data, such as offsets around parked vehicles, margins around bicycles, natural lines through curves, and stopping positions at intersections.

**Do not lean too far toward pure black-box End-to-End.**  
The system keeps BEV, Static World, Dynamic World, Agent Future, Dynamic Risk Map, candidate trajectories, an external evaluator, and loggable intermediate representations inside the architecture. This makes it possible to combine E2E learning capability with the verifiability, explainability, and safety monitorability required for a product.

**Base the architecture on tri-modal sensor fusion.**  
The design uses the complementary strengths of Vision (semantics, color, texture), LiDAR (shape, distance), and Radar (velocity, robustness in poor weather), avoiding dependence on a single sensor.

**Consider productization from the design stage.**  
The external evaluator, ODD monitoring, log design, Shadow Mode, regulatory response, and model lightweighting are built into the architecture design rather than added afterward.

---

## What This Book Does Not Cover

This book presents a design concept and does not cover the following:

- Evaluation of specific hardware products
- Detailed code explanation of specific OSS projects
- Legal interpretation of regulations (it discusses design policy for regulatory response, but does not provide legal advice)
- Distribution of a complete trained model

---

## Overall Structure

```text
Chapter 1  Design Philosophy: From ADAS to Structured End-to-End
Chapter 2  System Requirements and Overall Architecture
Chapter 3  Tri-modal BEV World Model
Chapter 4  Dynamic Object World Model and Future Risk Estimation
Chapter 5  Language Conditioning and VLA Planner
Chapter 6  Human Trajectory Teacher and Lane-Center-Free Planning
Chapter 7  Converting Selected Trajectories to Target Steering Angle
Chapter 8  Lightweight and Real-Time Design for Real-Vehicle Application
Chapter 9  Productization, Safety, Regulations, ODD, and Logging
Chapter 10 Implementation Modules and Interface Design
Chapter 11 Implementation Roadmap and Shadow Mode Validation
Chapter 12 Data Collection, Fleet Learning, and Annotation Pipeline [New]
Chapter 13 Evaluation Metrics, Benchmarks, and Scenario Design [New]
Chapter 14 Hardware Platform, Quantization, and OTA Deployment [New]
Appendix A  Forward Pseudocode
Appendix B  Output Format Definition
Appendix C  Reference OSS and Papers
Appendix D  Training Strategy and Use of OSS Public Weights
Appendix E  Glossary and Abbreviations
```
