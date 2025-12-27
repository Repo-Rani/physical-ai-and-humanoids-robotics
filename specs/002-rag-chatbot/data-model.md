# Data Model: RAG Chatbot for Physical AI & Humanoid Robotics Textbook

## Entity: Conversation
- **id**: UUID (Primary Key)
- **user_id**: UUID (Optional, for anonymous users)
- **created_at**: DateTime
- **updated_at**: DateTime
- **messages**: Array of Message objects
- **session_id**: String (for localStorage identification)

## Entity: Message
- **id**: UUID (Primary Key)
- **conversation_id**: UUID (Foreign Key)
- **sender_type**: Enum (USER | AI)
- **content**: String (message text)
- **timestamp**: DateTime
- **sources**: Array of Source objects (for AI responses)

## Entity: Source
- **id**: UUID (Primary Key)
- **url**: String (source document URL)
- **title**: String (document title)
- **score**: Float (relevance score from vector search)
- **text_preview**: String (text snippet)

## Entity: TextbookContent
- **id**: UUID (Primary Key)
- **url**: String (document URL)
- **title**: String (page title)
- **content**: String (chunked text content)
- **embedding**: Array of Float (vector embedding)
- **chunk_index**: Integer (order of chunk in document)
- **created_at**: DateTime

## Entity: PrivacyProfile
- **id**: UUID (Primary Key)
- **user_id**: UUID (Optional)
- **anonymized_data**: Boolean (whether data is anonymized)
- **retention_days**: Integer (days to retain data)
- **created_at**: DateTime

## Entity: ServiceDependency
- **id**: UUID (Primary Key)
- **name**: String (service name, e.g., "OpenRouter", "Qdrant", "Cohere")
- **status**: Enum (ACTIVE | FAILING | INACTIVE)
- **fallback_order**: Integer (priority for fallback)
- **last_check**: DateTime
- **created_at**: DateTime

## Entity: ChatRequest
- **message**: String (user query)
- **conversation_history**: Array of Message objects (last 10 messages)
- **model_index**: Integer (which model to try first)

## Entity: ChatResponse
- **response**: String (AI-generated response)
- **success**: Boolean (whether request succeeded)
- **sources**: Array of Source objects
- **model_used**: String (which model generated the response)