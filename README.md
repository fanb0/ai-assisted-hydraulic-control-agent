# AI-Assisted Hydraulic Control Agent

## Project Overview

This repository presents an AI-assisted Agent workflow for hydraulic manipulator control research, MATLAB/Simulink simulation debugging, and technical documentation.

The project focuses on using large language models and Agent-based workflows to support the research process of a three-degree-of-freedom hydraulic manipulator, including literature analysis, dynamic modeling, controller design, code debugging, simulation result interpretation, and report generation.

## Core Problem

Hydraulic manipulators are complex nonlinear mechatronic-hydraulic systems. The research process involves mechanical dynamics, hydraulic actuator pressure dynamics, servo valve flow characteristics, friction, leakage, external disturbances, disturbance observers, recurrent neural network controllers, and MATLAB/Simulink/Simscape simulation.

A single task often requires analyzing papers, equations, MATLAB/C++/Python code, simulation logs, control parameters, and experimental curves. Manual analysis is time-consuming and inefficient, especially when debugging complex simulation models or organizing long technical documents.

## AI Agent Workflow

The current workflow includes the following modules:

### 1. Literature Analysis Agent

This module is used to summarize research papers related to hydraulic manipulator modeling, disturbance rejection control, recurrent neural networks, output feedback control, and adaptive control methods. It helps extract key methods, technical routes, and differences between existing approaches.

### 2. Modeling Agent

This module assists in organizing the mathematical modeling process of the hydraulic manipulator, including mechanical dynamics, hydraulic actuator models, servo valve flow equations, and the coupling relationship between joint torque and hydraulic cylinder force.

### 3. Control Design Agent

This module supports the analysis and organization of disturbance observers, recurrent neural network approximation structures, command-filtered backstepping control, output feedback control, and controller stability analysis.

### 4. Code Debugging Agent

This module is used to analyze MATLAB/Simulink/Simscape errors, S-Function compilation issues, sampling time conflicts, control input saturation, parameter abnormalities, and abnormal simulation curves. It provides possible causes and debugging suggestions.

### 5. Documentation Agent

This module converts modeling processes, control strategies, simulation results, figure descriptions, and comparison analysis into structured technical documents, research notes, and thesis-style content.

## Current Results

The workflow has been applied to a three-degree-of-freedom hydraulic manipulator disturbance rejection control study. It has supported:

- Controller structure design
- Simulation model debugging
- MATLAB/Simulink error analysis
- Parameter adjustment suggestions
- Tracking error curve interpretation
- State estimation result explanation
- Comparative simulation analysis
- Technical document and thesis content organization

## Repository Structure

```text
docs/
  Project introduction and technical notes

workflow/
  AI Agent workflow diagrams

screenshots/
  AI usage records, billing records, and debugging screenshots

figures/
  Simulation results and comparison figures

code_examples/
  Desensitized MATLAB, Python, and C++ code examples
