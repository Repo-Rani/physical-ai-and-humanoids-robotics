# rag-design-skill

## Description

Provides reusable guidance for designing Retrieval-Augmented Generation (RAG) systems for AI-native textbooks and chatbots.

## Components

### Content Chunking Strategies
- Define optimal chunk sizes based on content type
- Implement semantic chunking for maintaining context
- Handle overlapping chunks to preserve continuity
- Consider document hierarchy and structure

### Embedding Generation Workflows
- Select appropriate embedding models for educational content
- Batch processing strategies for large textbook content
- Vector dimensionality considerations
- Embedding update and versioning strategies

### Retrieval and Ranking Approaches
- Hybrid search combining semantic and keyword-based retrieval
- Re-ranking strategies for improved relevance
- Context window optimization
- Multi-stage retrieval pipelines

### Source-Restricted Answering Logic
- Ensure responses are grounded in retrieved content
- Citation and source attribution mechanisms
- Confidence scoring for retrieved passages
- Fallback handling when sources are insufficient

## Responsibilities

### Guide Agents on Content Preparation for RAG
- Provide best practices for preprocessing textbook content
- Recommend metadata tagging strategies
- Define content quality checks before ingestion
- Suggest content normalization approaches

### Suggest Retrieval Strategies Without Implementation
- Recommend retrieval algorithms based on use case
- Provide architectural guidance for RAG pipelines
- Suggest evaluation metrics for retrieval quality
- Offer trade-off analysis between different approaches

### Ensure Answers Stay Grounded in Source Material
- Enforce source attribution requirements
- Implement answer verification against retrieved context
- Prevent hallucination through strict source adherence
- Provide guidelines for handling ambiguous queries

## Usage Guidelines

When this skill is invoked, agents should:

1. **Analyze the content structure** before recommending chunking strategies
2. **Consider the user query patterns** when designing retrieval approaches
3. **Prioritize accuracy over completeness** to maintain source fidelity
4. **Suggest evaluation methods** to measure RAG system effectiveness
5. **Document architectural decisions** for future reference

## Best Practices

- **Start simple**: Begin with basic retrieval before adding complexity
- **Measure everything**: Track retrieval accuracy, latency, and relevance
- **Iterate based on feedback**: Use user interactions to refine retrieval
- **Maintain source traceability**: Always link answers back to specific content sections
- **Handle edge cases**: Plan for queries that span multiple sources or have no good matches
