# Research Summary: RAG Chatbot Implementation

## Decision: FastAPI as Backend Framework
**Rationale**: FastAPI provides excellent performance, automatic API documentation, and strong typing support. It's ideal for the RAG pipeline with built-in async support for handling multiple requests efficiently.
**Alternatives considered**: Express.js, Flask, Django - FastAPI chosen for its performance characteristics and built-in OpenAPI generation.

## Decision: Qdrant as Vector Database
**Rationale**: Qdrant offers excellent performance for vector similarity search, supports multiple distance metrics, and provides both cloud and self-hosted options. It has good Python client support and integrates well with Cohere embeddings.
**Alternatives considered**: Pinecone, Weaviate, ChromaDB - Qdrant chosen for its balance of performance, features, and pricing model.

## Decision: Cohere for Embeddings
**Rationale**: Cohere provides high-quality embeddings with the embed-english-v3.0 model that has 1024 dimensions, which is efficient for vector search. It has good performance and reliability for educational content.
**Alternatives considered**: OpenAI embeddings, Hugging Face models - Cohere chosen for its domain-specific performance and cost-effectiveness.

## Decision: OpenRouter with Multiple Models
**Rationale**: OpenRouter provides access to multiple LLMs with a unified API. Using multiple fallback models (Google Gemini, Mistral, DeepSeek) ensures reliability when primary models are unavailable.
**Alternatives considered**: Direct OpenAI API, Anthropic Claude - OpenRouter chosen for its diverse model selection and fallback capabilities.

## Decision: Docusaurus Root Theme Integration
**Rationale**: Integrating via Docusaurus Root theme ensures the chatbot appears on all pages without modifying individual components. This provides consistent access to the AI tutor throughout the textbook.
**Alternatives considered**: Adding to specific components, separate chat page - Root theme chosen for universal accessibility.

## Decision: Browser LocalStorage for History
**Rationale**: Using localStorage provides persistent chat history across page reloads without requiring user accounts. It's simple to implement and works well for the educational use case.
**Alternatives considered**: Server-side storage, session storage - localStorage chosen for its simplicity and privacy compliance.