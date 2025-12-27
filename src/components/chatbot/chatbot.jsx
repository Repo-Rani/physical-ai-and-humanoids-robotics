import React, { useState, useRef, useEffect } from 'react';
import { MessageCircle, X, Send, Bot, User, Loader2, ExternalLink, Trash2, Sparkles } from 'lucide-react';
import { v4 as uuidv4 } from 'uuid';
import TextSelectionHandler from './TextSelectionHandler';

const HumanoidChatbot = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [selectedTextContext, setSelectedTextContext] = useState(null);
  const [conversationId] = useState(() => {
    const stored = localStorage.getItem('chatbot_conversation_id');
    if (stored) return stored;
    const newId = uuidv4();
    localStorage.setItem('chatbot_conversation_id', newId);
    return newId;
  });
  const [messages, setMessages] = useState(() => {
    const stored = localStorage.getItem('chatbot_messages');
    if (stored) {
      try {
        return JSON.parse(stored).map(msg => ({
          ...msg,
          timestamp: new Date(msg.timestamp)
        }));
      } catch {
        return [{
          type: 'ai',
          text: 'Welcome to Humanoid Robotics & Physical AI Learning! 🤖 Ask me anything about robotics, AI, or our documentation.',
          timestamp: new Date(),
          sources: []
        }];
      }
    }
    return [{
      type: 'ai',
      text: 'Welcome to Humanoid Robotics & Physical AI Learning! 🤖 Ask me anything about robotics, AI, or our documentation.',
      timestamp: new Date(),
      sources: []
    }];
  });
  const [input, setInput] = useState('');
  const [isTyping, setIsTyping] = useState(false);
  const messagesEndRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    localStorage.setItem('chatbot_messages', JSON.stringify(messages));
  }, [messages]);

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleSend = async (customMessage = null, useSelectedText = false) => {
    const messageToSend = typeof customMessage === 'string' 
      ? customMessage 
      : (customMessage || input);
    
    if (!messageToSend || typeof messageToSend !== 'string' || !messageToSend.trim() || isTyping) {
      return;
    }

    const userMessage = {
      type: 'user',
      text: messageToSend,
      timestamp: new Date(),
      sources: [],
      selectedContext: useSelectedText ? selectedTextContext : null
    };

    setMessages(prev => [...prev, userMessage]);
    const currentInput = messageToSend;
    setInput('');
    setIsTyping(true);

    try {
      const response = await fetch("http://localhost:8000/api/v1/chat", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({
          message: currentInput,
          conversation_id: conversationId,
          selected_text: useSelectedText ? selectedTextContext : null
        })
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();

      const aiMessage = {
        type: 'ai',
        text: data.response || 'I apologize, but I encountered an error. Please try again.',
        timestamp: new Date(data.timestamp),
        sources: data.sources || []
      };

      setMessages(prev => [...prev, aiMessage]);
    } catch (error) {
      console.error('Chat error:', error);
      const errorMessage = {
        type: 'ai',
        text: '⚠️ Sorry, I am currently unable to process your request. Please make sure the backend server is running on http://localhost:8000',
        timestamp: new Date(),
        sources: []
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsTyping(false);
      if (useSelectedText) {
        setSelectedTextContext(null);
      }
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSend();
    }
  };

  const handleClearHistory = () => {
    if (window.confirm('Are you sure you want to clear all chat history? This action cannot be undone.')) {
      setMessages([{
        type: 'ai',
        text: 'Welcome to Humanoid Robotics & Physical AI Learning! 🤖 Ask me anything about robotics, AI, or our documentation.',
        timestamp: new Date(),
        sources: []
      }]);

      localStorage.removeItem('chatbot_messages');

      fetch("http://localhost:8000/api/v1/chat/clear", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({
          conversation_id: conversationId
        })
      }).catch(err => console.error('Failed to clear server history:', err));
    }
  };

  const handleTextSelection = (text) => {
    setSelectedTextContext(text);
    setIsOpen(true);
    const prefilledQuestion = `Can you explain this: "${text.length > 100 ? text.substring(0, 100) + '...' : text}"`;
    setInput(prefilledQuestion);
  };

  return (
    <>
      <TextSelectionHandler onTextSelected={handleTextSelection} />
      
      {/* FIXED: Proper z-index hierarchy */}
      <div className="chatbot-container" style={{
        position: 'fixed',
        bottom: '24px',
        right: '24px',
        zIndex: 9999,
        fontFamily: "'Inter', 'Poppins', sans-serif"
      }}>
        {/* Chatbot Window - FIXED positioning and z-index */}
        <div
          className={`chatbot-window ${isOpen ? 'chatbot-window-open' : 'chatbot-window-closed'}`}
          style={{
            position: 'absolute',
            bottom: '80px',
            right: '0',
            width: '400px',
            maxWidth: '90vw',
            height: '600px',
            maxHeight: '80vh',
            backgroundColor: 'white',
            borderRadius: '20px',
            boxShadow: '0 20px 60px rgba(0, 0, 0, 0.3)',
            border: '1px solid rgba(21, 128, 61, 0.2)',
            display: 'flex',
            flexDirection: 'column',
            overflow: 'hidden',
            transition: 'all 0.3s cubic-bezier(0.4, 0, 0.2, 1)',
            opacity: isOpen ? 1 : 0,
            transform: isOpen ? 'scale(1) translateY(0)' : 'scale(0.95) translateY(10px)',
            pointerEvents: isOpen ? 'auto' : 'none',
            zIndex: 10000
          }}
        >
          {/* Header */}
          <div style={{
            background: 'linear-gradient(135deg, #059669 0%, #14b8a6 100%)',
            padding: '20px',
            borderTopLeftRadius: '20px',
            borderTopRightRadius: '20px',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'space-between'
          }}>
            <div style={{ display: 'flex', alignItems: 'center', gap: '12px' }}>
              <div style={{ position: 'relative' }}>
                <div style={{
                  width: '44px',
                  height: '44px',
                  backgroundColor: 'white',
                  borderRadius: '50%',
                  display: 'flex',
                  alignItems: 'center',
                  justifyContent: 'center',
                  boxShadow: '0 4px 12px rgba(0,0,0,0.1)'
                }}>
                  <Bot style={{ width: '24px', height: '24px', color: '#059669' }} />
                </div>
                <div style={{
                  position: 'absolute',
                  bottom: '-2px',
                  right: '-2px',
                  width: '14px',
                  height: '14px',
                  backgroundColor: '#10b981',
                  borderRadius: '50%',
                  border: '2px solid white',
                  animation: 'pulse 2s ease-in-out infinite'
                }}></div>
              </div>
              <div>
                <h3 style={{ 
                  color: 'white', 
                  fontWeight: '600', 
                  fontSize: '16px',
                  margin: 0,
                  marginBottom: '2px'
                }}>
                  AI Robotics Tutor
                </h3>
                <div style={{ display: 'flex', alignItems: 'center', gap: '6px' }}>
                  <Sparkles style={{ width: '12px', height: '12px', color: '#a7f3d0' }} />
                  <p style={{ 
                    color: '#d1fae5', 
                    fontSize: '11px',
                    margin: 0,
                    fontWeight: '500'
                  }}>
                    Powered by RAG
                  </p>
                </div>
              </div>
            </div>
            <div style={{ display: 'flex', gap: '8px' }}>
              <button
                onClick={handleClearHistory}
                title="Clear chat history"
                style={{
                  background: 'rgba(255, 255, 255, 0.15)',
                  border: 'none',
                  borderRadius: '8px',
                  padding: '8px',
                  cursor: 'pointer',
                  display: 'flex',
                  alignItems: 'center',
                  justifyContent: 'center',
                  transition: 'background 0.2s',
                  color: 'white'
                }}
                onMouseEnter={(e) => e.target.style.background = 'rgba(255, 255, 255, 0.25)'}
                onMouseLeave={(e) => e.target.style.background = 'rgba(255, 255, 255, 0.15)'}
              >
                <Trash2 style={{ width: '16px', height: '16px' }} />
              </button>
              <button
                onClick={() => setIsOpen(false)}
                style={{
                  background: 'rgba(255, 255, 255, 0.15)',
                  border: 'none',
                  borderRadius: '8px',
                  padding: '8px',
                  cursor: 'pointer',
                  display: 'flex',
                  alignItems: 'center',
                  justifyContent: 'center',
                  transition: 'background 0.2s',
                  color: 'white'
                }}
                onMouseEnter={(e) => e.target.style.background = 'rgba(255, 255, 255, 0.25)'}
                onMouseLeave={(e) => e.target.style.background = 'rgba(255, 255, 255, 0.15)'}
              >
                <X style={{ width: '18px', height: '18px' }} />
              </button>
            </div>
          </div>

          {/* Messages Container */}
          <div style={{
            flex: 1,
            overflowY: 'auto',
            padding: '20px',
            background: 'linear-gradient(to bottom, #f9fafb 0%, #ffffff 100%)',
            display: 'flex',
            flexDirection: 'column',
            gap: '16px'
          }}>
            {messages.map((msg, idx) => (
              <div
                key={idx}
                style={{
                  display: 'flex',
                  gap: '12px',
                  flexDirection: msg.type === 'user' ? 'row-reverse' : 'row',
                  animation: 'fadeIn 0.3s ease-out'
                }}
              >
                {/* Avatar */}
                <div
                  style={{
                    width: '36px',
                    height: '36px',
                    borderRadius: '50%',
                    background: msg.type === 'ai'
                      ? 'linear-gradient(135deg, #059669 0%, #14b8a6 100%)'
                      : 'linear-gradient(135deg, #3b82f6 0%, #6366f1 100%)',
                    display: 'flex',
                    alignItems: 'center',
                    justifyContent: 'center',
                    flexShrink: 0,
                    boxShadow: '0 4px 12px rgba(0,0,0,0.1)'
                  }}
                >
                  {msg.type === 'ai' ? (
                    <Bot style={{ width: '20px', height: '20px', color: 'white' }} />
                  ) : (
                    <User style={{ width: '20px', height: '20px', color: 'white' }} />
                  )}
                </div>

                {/* Message Bubble */}
                <div style={{
                  display: 'flex',
                  flexDirection: 'column',
                  alignItems: msg.type === 'user' ? 'flex-end' : 'flex-start',
                  maxWidth: '75%'
                }}>
                  <div
                    style={{
                      borderRadius: '16px',
                      padding: '14px 18px',
                      boxShadow: msg.type === 'ai' 
                        ? '0 2px 8px rgba(0,0,0,0.08)' 
                        : '0 4px 12px rgba(59,130,246,0.3)',
                      background: msg.type === 'ai'
                        ? 'white'
                        : 'linear-gradient(135deg, #3b82f6 0%, #6366f1 100%)',
                      border: msg.type === 'ai' ? '1px solid #e5e7eb' : 'none'
                    }}
                  >
                    {msg.type === 'ai' && (
                      <div style={{
                        display: 'flex',
                        alignItems: 'center',
                        gap: '8px',
                        marginBottom: '10px',
                        paddingBottom: '10px',
                        borderBottom: '1px solid #e5e7eb'
                      }}>
                        <div style={{
                          width: '8px',
                          height: '8px',
                          backgroundColor: '#10b981',
                          borderRadius: '50%',
                          animation: 'pulse 2s ease-in-out infinite'
                        }}></div>
                        <span style={{
                          fontSize: '11px',
                          fontWeight: '600',
                          color: '#059669',
                          textTransform: 'uppercase',
                          letterSpacing: '0.5px'
                        }}>
                          AI Assistant
                        </span>
                      </div>
                    )}
                    <p style={{
                      fontSize: '14px',
                      lineHeight: '1.6',
                      margin: 0,
                      color: msg.type === 'ai' ? '#1f2937' : 'white',
                      whiteSpace: 'pre-wrap',
                      wordBreak: 'break-word'
                    }}>
                      {msg.text}
                    </p>

                    {/* Citations */}
                    {msg.sources && msg.sources.length > 0 && (
                      <div style={{
                        marginTop: '12px',
                        paddingTop: '12px',
                        borderTop: '1px solid #e5e7eb'
                      }}>
                        <p style={{
                          fontSize: '11px',
                          fontWeight: '600',
                          color: '#059669',
                          marginBottom: '8px',
                          textTransform: 'uppercase',
                          letterSpacing: '0.5px'
                        }}>
                          📚 Sources
                        </p>
                        <div style={{ display: 'flex', flexDirection: 'column', gap: '8px' }}>
                          {msg.sources.map((source, idx) => (
                            <a
                              key={idx}
                              href={source.url}
                              target="_blank"
                              rel="noopener noreferrer"
                              style={{
                                display: 'flex',
                                alignItems: 'flex-start',
                                gap: '8px',
                                fontSize: '12px',
                                color: '#2563eb',
                                textDecoration: 'none',
                                padding: '8px',
                                borderRadius: '8px',
                                background: '#f0f9ff',
                                transition: 'background 0.2s'
                              }}
                              onMouseEnter={(e) => e.target.style.background = '#dbeafe'}
                              onMouseLeave={(e) => e.target.style.background = '#f0f9ff'}
                            >
                              <ExternalLink style={{ width: '14px', height: '14px', marginTop: '2px', flexShrink: 0 }} />
                              <span>
                                <span style={{ fontWeight: '600', display: 'block' }}>{source.title}</span>
                                {source.snippet && (
                                  <span style={{ color: '#6b7280', fontStyle: 'italic', display: 'block', marginTop: '4px' }}>
                                    {source.snippet}
                                  </span>
                                )}
                              </span>
                            </a>
                          ))}
                        </div>
                      </div>
                    )}
                  </div>
                  <span style={{
                    fontSize: '11px',
                    color: '#9ca3af',
                    marginTop: '6px',
                    paddingLeft: msg.type === 'user' ? '0' : '8px',
                    paddingRight: msg.type === 'user' ? '8px' : '0'
                  }}>
                    {msg.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
                  </span>
                </div>
              </div>
            ))}

            {isTyping && (
              <div style={{
                display: 'flex',
                gap: '12px',
                animation: 'fadeIn 0.3s ease-out'
              }}>
                <div style={{
                  width: '36px',
                  height: '36px',
                  borderRadius: '50%',
                  background: 'linear-gradient(135deg, #059669 0%, #14b8a6 100%)',
                  display: 'flex',
                  alignItems: 'center',
                  justifyContent: 'center',
                  boxShadow: '0 4px 12px rgba(0,0,0,0.1)'
                }}>
                  <Loader2 style={{ width: '20px', height: '20px', color: 'white', animation: 'spin 1s linear infinite' }} />
                </div>
                <div style={{
                  backgroundColor: 'white',
                  border: '1px solid #e5e7eb',
                  borderRadius: '16px',
                  padding: '14px 18px',
                  boxShadow: '0 2px 8px rgba(0,0,0,0.08)'
                }}>
                  <div style={{ display: 'flex', gap: '6px', alignItems: 'center' }}>
                    <div style={{
                      width: '8px',
                      height: '8px',
                      backgroundColor: '#10b981',
                      borderRadius: '50%',
                      animation: 'bounce 1.4s ease-in-out 0s infinite'
                    }}></div>
                    <div style={{
                      width: '8px',
                      height: '8px',
                      backgroundColor: '#10b981',
                      borderRadius: '50%',
                      animation: 'bounce 1.4s ease-in-out 0.2s infinite'
                    }}></div>
                    <div style={{
                      width: '8px',
                      height: '8px',
                      backgroundColor: '#10b981',
                      borderRadius: '50%',
                      animation: 'bounce 1.4s ease-in-out 0.4s infinite'
                    }}></div>
                    <span style={{ fontSize: '12px', color: '#6b7280', marginLeft: '8px' }}>
                      Searching knowledge base...
                    </span>
                  </div>
                </div>
              </div>
            )}

            <div ref={messagesEndRef} />
          </div>

          {/* Input Area */}
          <div style={{
            padding: '16px 20px',
            backgroundColor: 'white',
            borderTop: '1px solid #e5e7eb',
            borderBottomLeftRadius: '20px',
            borderBottomRightRadius: '20px'
          }}>
            {selectedTextContext && (
              <div style={{
                background: 'linear-gradient(135deg, #dbeafe 0%, #e0e7ff 100%)',
                padding: '10px 12px',
                borderRadius: '10px',
                marginBottom: '12px',
                fontSize: '12px',
                color: '#1e40af',
                display: 'flex',
                alignItems: 'center',
                gap: '8px'
              }}>
                <Sparkles style={{ width: '14px', height: '14px', flexShrink: 0 }} />
                <span style={{ flex: 1, fontWeight: '500' }}>
                  Selected text will be used as context
                </span>
                <button
                  onClick={() => setSelectedTextContext(null)}
                  style={{
                    background: 'transparent',
                    border: 'none',
                    cursor: 'pointer',
                    padding: '4px',
                    display: 'flex',
                    alignItems: 'center',
                    color: '#1e40af'
                  }}
                >
                  <X style={{ width: '14px', height: '14px' }} />
                </button>
              </div>
            )}
            <div style={{ display: 'flex', gap: '10px' }}>
              <input
                type="text"
                value={input}
                onChange={(e) => setInput(e.target.value)}
                onKeyPress={handleKeyPress}
                placeholder="Ask about robotics, AI, simulation..."
                disabled={isTyping}
                style={{
                  flex: 1,
                  padding: '14px 16px',
                  backgroundColor: '#f9fafb',
                  color: '#1f2937',
                  borderRadius: '12px',
                  border: '1px solid #e5e7eb',
                  fontSize: '14px',
                  outline: 'none',
                  transition: 'all 0.2s',
                  opacity: isTyping ? 0.6 : 1
                }}
                onFocus={(e) => {
                  e.target.style.borderColor = '#059669';
                  e.target.style.backgroundColor = 'white';
                }}
                onBlur={(e) => {
                  e.target.style.borderColor = '#e5e7eb';
                  e.target.style.backgroundColor = '#f9fafb';
                }}
              />
              <button
                onClick={() => handleSend(null, !!selectedTextContext)}
                disabled={!input.trim() || isTyping}
                style={{
                  background: 'linear-gradient(135deg, #059669 0%, #14b8a6 100%)',
                  color: 'white',
                  padding: '14px',
                  borderRadius: '12px',
                  border: 'none',
                  cursor: isTyping || !input.trim() ? 'not-allowed' : 'pointer',
                  display: 'flex',
                  alignItems: 'center',
                  justifyContent: 'center',
                  transition: 'all 0.2s',
                  opacity: isTyping || !input.trim() ? 0.5 : 1,
                  boxShadow: isTyping || !input.trim() ? 'none' : '0 4px 12px rgba(5,150,105,0.3)',
                  minWidth: '48px'
                }}
                onMouseEnter={(e) => {
                  if (!isTyping && input.trim()) {
                    e.target.style.transform = 'scale(1.05)';
                  }
                }}
                onMouseLeave={(e) => {
                  e.target.style.transform = 'scale(1)';
                }}
              >
                {isTyping ? (
                  <Loader2 style={{ width: '20px', height: '20px', animation: 'spin 1s linear infinite' }} />
                ) : (
                  <Send style={{ width: '20px', height: '20px' }} />
                )}
              </button>
            </div>
            <p style={{
              fontSize: '11px',
              color: '#9ca3af',
              textAlign: 'center',
              marginTop: '10px',
              marginBottom: 0
            }}>
              Powered by Cohere + Qdrant + Gemini
            </p>
          </div>
        </div>

        {/* Floating Button - FIXED z-index */}
        <button
          onClick={() => setIsOpen(!isOpen)}
          style={{
            width: '64px',
            height: '64px',
            background: 'linear-gradient(135deg, #059669 0%, #14b8a6 100%)',
            borderRadius: '50%',
            border: 'none',
            boxShadow: '0 8px 24px rgba(5,150,105,0.4)',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center',
            cursor: 'pointer',
            transition: 'all 0.3s cubic-bezier(0.4, 0, 0.2, 1)',
            animation: 'float 3s ease-in-out infinite',
            position: 'relative',
            zIndex: 10001
          }}
          onMouseEnter={(e) => {
            e.target.style.transform = 'scale(1.1)';
            e.target.style.boxShadow = '0 12px 32px rgba(5,150,105,0.5)';
          }}
          onMouseLeave={(e) => {
            e.target.style.transform = 'scale(1)';
            e.target.style.boxShadow = '0 8px 24px rgba(5,150,105,0.4)';
          }}
        >
          {isOpen ? (
            <X style={{ width: '32px', height: '32px', color: 'white' }} />
          ) : (
            <>
              <MessageCircle style={{ width: '32px', height: '32px', color: 'white' }} />
              <div style={{
                position: 'absolute',
                top: '-4px',
                right: '-4px',
                width: '16px',
                height: '16px',
                backgroundColor: '#ef4444',
                borderRadius: '50%',
                animation: 'pulse 2s ease-in-out infinite',
                border: '2px solid white'
              }}></div>
            </>
          )}
        </button>

        <style>{`
          @keyframes float {
            0%, 100% {
              transform: translateY(0px);
            }
            50% {
              transform: translateY(-10px);
            }
          }

          @keyframes fadeIn {
            from {
              opacity: 0;
              transform: translateY(10px);
            }
            to {
              opacity: 1;
              transform: translateY(0);
            }
          }

          @keyframes pulse {
            0%, 100% {
              opacity: 1;
              transform: scale(1);
            }
            50% {
              opacity: 0.7;
              transform: scale(1.1);
            }
          }

          @keyframes spin {
            from {
              transform: rotate(0deg);
            }
            to {
              transform: rotate(360deg);
            }
          }

          @keyframes bounce {
            0%, 80%, 100% {
              transform: translateY(0);
            }
            40% {
              transform: translateY(-6px);
            }
          }

          .chatbot-window::-webkit-scrollbar {
            width: 6px;
          }

          .chatbot-window::-webkit-scrollbar-track {
            background: rgba(5, 150, 105, 0.05);
            border-radius: 10px;
          }

          .chatbot-window::-webkit-scrollbar-thumb {
            background: linear-gradient(to bottom, #059669, #14b8a6);
            border-radius: 10px;
          }

          .chatbot-window::-webkit-scrollbar-thumb:hover {
            background: linear-gradient(to bottom, #047857, #0d9488);
          }
        `}</style>
      </div>
    </>
  );
};

export default HumanoidChatbot;