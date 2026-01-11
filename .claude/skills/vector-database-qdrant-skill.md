# vector-database-qdrant-skill

## Description

Provides reusable design guidance for vector databases using Qdrant.

## Components

### Collection and Payload Design
- Define collection structure for optimal query patterns
- Design payload schemas for educational content metadata
- Implement versioning strategies for collections
- Handle multi-tenant collection architectures
- Define point IDs and naming conventions

### Metadata Filtering Strategies
- Design filterable payload fields for precise retrieval
- Implement hierarchical filtering (chapter, section, topic)
- Use keyword arrays for multi-dimensional filtering
- Optimize filter combinations for common queries
- Balance filter specificity with retrieval recall

### Similarity Search Tuning
- Configure distance metrics (cosine, euclidean, dot product)
- Optimize search parameters (limit, score_threshold)
- Implement HNSW index tuning for speed vs accuracy
- Design multi-stage search with rescoring
- Handle edge cases (no results, low-confidence matches)

### Cost-Efficient Storage Patterns
- Optimize vector dimensionality without sacrificing quality
- Implement quantization strategies for storage savings
- Design efficient payload structures to minimize storage
- Use disk-based storage for large-scale collections
- Implement archival strategies for historical data

## Responsibilities

### Advise on Qdrant Schema Design

**Collection Structure:**
- Recommend single vs multiple collection strategies
- Define when to use collection aliases
- Suggest partitioning strategies for large datasets
- Provide guidance on collection configuration parameters

**Payload Schema:**
- Design metadata fields for filtering and display
- Recommend indexed vs non-indexed fields
- Define data types for optimal storage and query performance
- Suggest nested payload structures when appropriate

**Point Management:**
- Define point ID generation strategies
- Recommend batch upsert patterns
- Suggest point deletion and update strategies
- Handle duplicate detection approaches

### Recommend Search and Filtering Strategies

**Basic Retrieval:**
- Simple similarity search for general queries
- Top-k retrieval with configurable limits
- Score threshold strategies for quality control
- Distance metric selection based on use case

**Advanced Filtering:**
- Combine vector search with metadata filters
- Use compound filters (AND, OR, NOT logic)
- Implement range filters for numerical metadata
- Apply geo-filters if location data is relevant

**Query Optimization:**
- Pre-filter vs post-filter trade-offs
- Batch query strategies for efficiency
- Pagination approaches for large result sets
- Caching strategies for common queries

### Optimize Retrieval Performance Conceptually

**Index Optimization:**
- HNSW parameters (M, ef_construct) tuning guidelines
- Balance between indexing time and search speed
- Memory vs disk storage trade-offs
- Index rebuild strategies for evolving data

**Query Performance:**
- Reduce vector dimensionality through PCA/compression
- Implement approximate nearest neighbor (ANN) strategies
- Use query result caching for repeated searches
- Optimize payload field selection to reduce transfer size

**Scalability Patterns:**
- Horizontal scaling with sharding
- Read replica strategies for high query loads
- Backup and restore strategies
- Monitoring and alerting for performance degradation

## Usage Guidelines

When this skill is invoked, agents should:

1. **Understand the data model** before recommending collection structure
2. **Analyze query patterns** to optimize filtering and search strategies
3. **Consider scale requirements** when suggesting storage and indexing approaches
4. **Balance performance and cost** in all recommendations
5. **Provide migration paths** when suggesting schema changes

## Best Practices

### Collection Design
- **Start with a single collection** and split only when necessary
- **Design payload schema upfront** to avoid costly migrations
- **Use consistent naming conventions** for fields and collections
- **Version your collections** for schema evolution

### Search Optimization
- **Profile your queries** to identify bottlenecks
- **Use filters early** in the query pipeline to reduce search space
- **Set appropriate score thresholds** to balance precision and recall
- **Monitor search latency** and adjust HNSW parameters accordingly

### Metadata Strategy
- **Index frequently filtered fields** for fast query performance
- **Keep payload lightweight** to reduce storage and transfer costs
- **Use arrays for multi-valued attributes** (tags, categories)
- **Include source references** for traceability back to original content

### Performance Monitoring
- **Track query latency** at p50, p95, and p99 percentiles
- **Monitor index build times** as data grows
- **Measure recall and precision** for search quality
- **Set up alerts** for degraded performance

## Qdrant-Specific Recommendations

### When to Use Qdrant
- Need for advanced filtering combined with vector search
- Require high-performance similarity search at scale
- Want built-in support for geo-filtering and hybrid search
- Need production-ready vector database with clustering support

### Key Advantages
- **Hybrid search**: Combine vector and metadata filtering efficiently
- **Filtering performance**: Qdrant's payload indexing is highly optimized
- **Scalability**: Built-in sharding and replication support
- **API flexibility**: REST and gRPC interfaces available

### Integration Patterns
- **Direct API integration** for custom applications
- **LangChain integration** for LLM applications
- **Batch processing** for large-scale ingestion
- **Streaming updates** for real-time content additions
