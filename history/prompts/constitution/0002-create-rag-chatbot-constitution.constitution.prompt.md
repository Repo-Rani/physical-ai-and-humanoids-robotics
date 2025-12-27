---
id: 0002
title: Create RAG Chatbot Constitution
stage: constitution
date: 2025-12-23
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: 002-rag-chatbot
user: user
command: /sp.constitution
labels: ["constitution", "rag-chatbot", "hackathon-requirements"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/rag-constitution.md
tests:
  - null
---

## Prompt

---
id: 0001
title: Update RAG Chatbot Constitution
stage: constitution
date: 2025-12-05
surface: agent
model: gemini-1.5-pro
feature: none
branch: main
user: user
command: /sp.constitution

# RAG CHATBOT IMPLEMENTATION CONSTITUTION - FRESH START

## 🎯 PROJECT GOAL
Build a Retrieval-Augmented Generation (RAG) chatbot from scratch and embed it in your Docusaurus Physical AI textbook. The chatbot will answer questions about the book content using OpenAI, Qdrant vector database, and Neon Postgres.

## 📋 REQUIREMENTS CHECKLIST
### Core Technologies (Non-Negotiable)
- ✅ OpenAI API - For embeddings and chat completion
- ✅ FastAPI - Backend framework
- ✅ Qdrant Cloud - Vector database (free tier)
- ✅ Neon Serverless Postgres - SQL database
- ✅ Docusaurus - Frontend (already exists)

### Key Features
1. ✅ Answer questions about book content
2. ✅ Support text selection queries
3. ✅ Embedded chatbot widget (bottom-right corner)
4. ✅ Real-time AI responses (not hardcoded)
5. ✅ Source citations from book chapters

## CORE REQUIREMENT
2. Integrated RAG Chatbot Development: Build and embed a Retrieval-Augmented Generation (RAG) chatbot within the published book. This chatbot, utilizing the OpenAI Agents/ChatKit SDKs, FastAPI, Neon Serverless Postgres database, and Qdrant Cloud Free Tier, must be able to answer user questions about the book's content, including answering questions based only on text selected by the user.

**User Request:** Create comprehensive constitution for RAG chatbot and add to `.specify/memory/rag-constitution.md` file. Do not change any other file.

## Response snapshot

Created comprehensive RAG Chatbot Constitution (v1.0.0) at `.specify/memory/rag-constitution.md` with 13 binding principles:

**Core Behavioral Guarantees:**
1. Selected-Text-Only Answering Mode (Strict Enforcement) - Treats selected text as single source of truth, no external knowledge
2. Hallucination Control & Refusal Policy - Answers grounded only in retrieved context, refuses when insufficient
3. Source Attribution Contract - Every answer must include chapter citations

**Conversation & Memory:**
4. Conversation Persistence (Neon Postgres) - Multi-turn conversation state management
5. Stateless vs Stateful RAG Modes - Clear separation, no context leakage

**Data Integrity:**
6. Content Indexing Contract - Complete metadata (text, chapter, source_path, chunk_index) for all vectors
7. Re-indexing Safety Rules - Explicit manual trigger, async execution, transparent error reporting

**Security:**
8. Environment Isolation - No exposed API keys, secrets from .env only
9. Cross-Origin Safety - Explicit CORS origins, no wildcards in production

**Future-Proofing:**
10. Authentication Compatibility - User identity injectable without architectural changes
11. Evaluation & Demo Readiness - 90-second demo showcase requirement

**Technical Standards:**
12. Technical Stack & Architecture - FastAPI, Qdrant, Neon Postgres, OpenAI, service-oriented design
13. Implementation Phases & Deliverables - 8 phases with testable deliverables

**Key Constitutional Constraints:**
- Selected text queries MUST NOT use vector search (hard requirement)
- Refusals are correct responses when context insufficient
- All answers require source citations
- No API key exposure (disqualification risk)
- Phased implementation with validation checkpoints

File includes Sync Impact Report, governance procedures, and compliance statement aligned with hackathon requirements.

## Outcome

- ✅ Impact: Created comprehensive 13-principle RAG constitution governing chatbot behavior, security, and implementation approach
- 🧪 Tests: No automated tests (constitution document); validation through manual compliance review at each phase
- 📁 Files: 1 new file created - `.specify/memory/rag-constitution.md` (v1.0.0)
- 🔁 Next prompts:
  - `/sp.specify` - Create feature specification for RAG chatbot implementation
  - `/sp.plan` - Develop architectural plan based on constitution principles
  - Review and approve constitution before proceeding to implementation
- 🧠 Reflection: Constitution provides clear behavioral contracts (especially selected-text mode and hallucination control) that are critical for hackathon evaluation. Principle 11 (Demo Readiness) ensures judges can verify compliance within 90 seconds. All 13 principles are binding and testable.

## Evaluation notes (flywheel)

- Failure modes observed: None - Constitution creation successful
- Graders run and results (PASS/FAIL): N/A (document creation, no automated grading)
- Prompt variant (if applicable): N/A (initial constitution)
- Next experiment (smallest change to try): Test constitution enforcement during Phase 1 backend setup - verify principle violations caught early
