import React, { useState, useRef, useEffect } from 'react';
import { MessageCircle, X, Send, Bot, User, Loader2, ExternalLink, Trash2, Sparkles } from 'lucide-react';
import { v4 as uuidv4 } from 'uuid';
import TextSelectionHandler from './TextSelectionHandler';

const HumanoidChatbot = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [selectedTextContext, setSelectedTextContext] = useState(null);
  const [input, setInput] = useState('');
  const [isTyping, setIsTyping] = useState(false);
  const messagesEndRef = useRef(null);
  
  const [isClient, setIsClient] = useState(false);
  const [conversationId, setConversationId] = useState(null);
  const [messages, setMessages] = useState([{
    type: 'ai',
    text: 'Welcome to Humanoid Robotics & Physical AI Learning! 🤖 Ask me anything about robotics, AI, or our documentation.',
    timestamp: new Date(),
    sources: []
  }]);

  useEffect(() => {
    setIsClient(true);
    
    const storedId = localStorage.getItem('chatbot_conversation_id');
    if (storedId) {
      setConversationId(storedId);
    } else {
      const newId = uuidv4();
      localStorage.setItem('chatbot_conversation_id', newId);
      setConversationId(newId);
    }

    const storedMessages = localStorage.getItem('chatbot_messages');
    if (storedMessages) {
      try {
        const parsedMessages = JSON.parse(storedMessages).map(msg => ({
          ...msg,
          timestamp: new Date(msg.timestamp)
        }));
        setMessages(parsedMessages);
      } catch (error) {
        console.error('Failed to parse stored messages:', error);
      }
    }
  }, []);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    if (isClient && messages.length > 0) {
      localStorage.setItem('chatbot_messages', JSON.stringify(messages));
    }
  }, [messages, isClient]);

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

      if (isClient) {
        localStorage.removeItem('chatbot_messages');
      }

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

  if (!isClient) {
    return null;
  }

  return (
    <>
      <TextSelectionHandler onTextSelected={handleTextSelection} />
      
      <div className="chatbot-container" style={{
        position: 'fixed',
        bottom: '24px',
        right: '24px',
        zIndex: 9999,
        fontFamily: "'Inter', 'Segoe UI', system-ui, sans-serif"
      }}>
        {/* Chatbot Window */}
        <div
          className={`chatbot-window ${isOpen ? 'chatbot-window-open' : 'chatbot-window-closed'}`}
          style={{
            position: 'absolute',
            bottom: '80px',
            right: '0',
            width: '420px',
            maxWidth: '90vw',
            height: '620px',
            maxHeight: '85vh',
            backgroundColor: 'var(--hb-black)',
            borderRadius: '24px',
            boxShadow: '0 25px 70px rgba(21, 128, 61, 0.25), 0 10px 30px rgba(0, 0, 0, 0.15), inset 0 1px 0 rgba(255, 255, 255, 0.1)',
            border: '1px solid rgba(21, 128, 61, 0.3)',
            display: 'flex',
            flexDirection: 'column',
            overflow: 'hidden',
            transition: 'all 0.4s cubic-bezier(0.68, -0.55, 0.265, 1.55)',
            opacity: isOpen ? 1 : 0,
            transform: isOpen ? 'scale(1) translateY(0) rotateX(0deg)' : 'scale(0.9) translateY(20px) rotateX(-10deg)',
            pointerEvents: isOpen ? 'auto' : 'none',
            zIndex: 10000,
            backdropFilter: 'blur(10px)'
          }}
        >
          {/* Header */}
          <div style={{
            background: 'linear-gradient(135deg, #15803d 0%, #16a34a 50%, #14b8a6 100%)',
            padding: '24px',
            borderTopLeftRadius: '24px',
            borderTopRightRadius: '24px',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'space-between',
            position: 'relative',
            overflow: 'hidden'
          }}>
            {/* Animated background effect */}
            <div style={{
              position: 'absolute',
              top: 0,
              left: 0,
              right: 0,
              bottom: 0,
              background: 'radial-gradient(circle at 30% 50%, rgba(255, 255, 255, 0.1) 0%, transparent 60%)',
              animation: 'shimmer 3s ease-in-out infinite'
            }}></div>
            
            <div style={{ display: 'flex', alignItems: 'center', gap: '14px', position: 'relative', zIndex: 1 }}>
              <div style={{ position: 'relative' }}>
                <div style={{
                  width: '50px',
                  height: '50px',
                  background: 'linear-gradient(135deg, #ffffff 0%, #f0fdf4 100%)',
                  borderRadius: '50%',
                  display: 'flex',
                  alignItems: 'center',
                  justifyContent: 'center',
                  boxShadow: '0 8px 20px rgba(0, 0, 0, 0.2), 0 0 0 4px rgba(255, 255, 255, 0.2)',
                  animation: 'avatarPulse 2s ease-in-out infinite'
                }}>
                  <Bot style={{ width: '28px', height: '28px', color: '#15803d' }} />
                </div>
                <div style={{
                  position: 'absolute',
                  bottom: '0',
                  right: '0',
                  width: '16px',
                  height: '16px',
                  background: 'linear-gradient(135deg, #22c55e 0%, #16a34a 100%)',
                  borderRadius: '50%',
                  border: '3px solid white',
                  boxShadow: '0 2px 8px rgba(34, 197, 94, 0.5)',
                  animation: 'statusBlink 2s ease-in-out infinite'
                }}></div>
              </div>
              <div>
                <h3 style={{ 
                  color: 'white', 
                  fontWeight: '700', 
                  fontSize: '17px',
                  margin: 0,
                  marginBottom: '4px',
                  textShadow: '0 2px 4px rgba(0, 0, 0, 0.2)',
                  letterSpacing: '-0.3px'
                }}>
                  AI Robotics Tutor
                </h3>
                <div style={{ display: 'flex', alignItems: 'center', gap: '8px' }}>
                  <Sparkles style={{ width: '13px', height: '13px', color: '#d1fae5', animation: 'sparkle 1.5s ease-in-out infinite' }} />
                  <p style={{ 
                    color: '#d1fae5', 
                    fontSize: '12px',
                    margin: 0,
                    fontWeight: '600',
                    textShadow: '0 1px 2px rgba(0, 0, 0, 0.2)'
                  }}>
                    Powered by RAG
                  </p>
                </div>
              </div>
            </div>
            <div style={{ display: 'flex', gap: '10px', position: 'relative', zIndex: 1 }}>
              <button
                onClick={handleClearHistory}
                title="Clear chat history"
                className="header-button"
                style={{
                  background: 'rgba(255, 255, 255, 0.18)',
                  backdropFilter: 'blur(10px)',
                  border: '1px solid rgba(255, 255, 255, 0.2)',
                  borderRadius: '10px',
                  padding: '10px',
                  cursor: 'pointer',
                  display: 'flex',
                  alignItems: 'center',
                  justifyContent: 'center',
                  transition: 'all 0.3s cubic-bezier(0.4, 0, 0.2, 1)',
                  color: 'white'
                }}
              >
                <Trash2 style={{ width: '18px', height: '18px' }} />
              </button>
              <button
                onClick={() => setIsOpen(false)}
                className="header-button"
                style={{
                  background: 'rgba(255, 255, 255, 0.18)',
                  backdropFilter: 'blur(10px)',
                  border: '1px solid rgba(255, 255, 255, 0.2)',
                  borderRadius: '10px',
                  padding: '10px',
                  cursor: 'pointer',
                  display: 'flex',
                  alignItems: 'center',
                  justifyContent: 'center',
                  transition: 'all 0.3s cubic-bezier(0.4, 0, 0.2, 1)',
                  color: 'white'
                }}
              >
                <X style={{ width: '20px', height: '20px' }} />
              </button>
            </div>
          </div>

          {/* Messages Container */}
          <div style={{
            flex: 1,
            overflowY: 'auto',
            padding: '24px',
            background: 'var(--hb-black)',
            display: 'flex',
            flexDirection: 'column',
            gap: '18px',
            position: 'relative'
          }}>
            {/* Decorative background pattern */}
            <div style={{
              position: 'absolute',
              top: 0,
              left: 0,
              right: 0,
              bottom: 0,
              backgroundImage: 'radial-gradient(circle at 20px 20px, rgba(21, 128, 61, 0.03) 1px, transparent 1px)',
              backgroundSize: '40px 40px',
              pointerEvents: 'none',
              opacity: 0.5
            }}></div>

            {messages.map((msg, idx) => (
              <div
                key={idx}
                style={{
                  display: 'flex',
                  gap: '14px',
                  flexDirection: msg.type === 'user' ? 'row-reverse' : 'row',
                  animation: 'slideIn 0.4s cubic-bezier(0.4, 0, 0.2, 1)',
                  animationDelay: `${idx * 0.05}s`,
                  position: 'relative',
                  zIndex: 1
                }}
              >
                {/* Avatar */}
                <div
                  style={{
                    width: '40px',
                    height: '40px',
                    borderRadius: '50%',
                    background: msg.type === 'ai'
                      ? 'linear-gradient(135deg, #15803d 0%, #16a34a 100%)'
                      : 'linear-gradient(135deg, #0d5225 0%, #15803d 100%)',
                    display: 'flex',
                    alignItems: 'center',
                    justifyContent: 'center',
                    flexShrink: 0,
                    boxShadow: msg.type === 'ai' 
                      ? '0 8px 20px rgba(21, 128, 61, 0.35), 0 0 0 3px rgba(21, 128, 61, 0.1)'
                      : '0 8px 20px rgba(13, 82, 37, 0.35), 0 0 0 3px rgba(13, 82, 37, 0.1)',
                    border: '2px solid rgba(255, 255, 255, 0.2)',
                    animation: 'avatarBounce 0.6s cubic-bezier(0.68, -0.55, 0.265, 1.55)'
                  }}
                >
                  {msg.type === 'ai' ? (
                    <Bot style={{ width: '22px', height: '22px', color: 'white' }} />
                  ) : (
                    <User style={{ width: '22px', height: '22px', color: 'white' }} />
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
                      borderRadius: msg.type === 'user' ? '20px 20px 4px 20px' : '20px 20px 20px 4px',
                      padding: '16px 20px',
                      boxShadow: msg.type === 'ai' 
                        ? '0 4px 15px rgba(21, 128, 61, 0.12), 0 1px 3px rgba(0, 0, 0, 0.08)' 
                        : '0 4px 15px rgba(13, 82, 37, 0.25), 0 1px 3px rgba(0, 0, 0, 0.1)',
                      background: msg.type === 'ai'
                        ? 'var(--hb-black)'
                        : 'linear-gradient(135deg, #15803d 0%, #16a34a 100%)',
                      border: msg.type === 'ai' ? '1px solid rgba(21, 128, 61, 0.15)' : 'none',
                      position: 'relative',
                      overflow: 'hidden',
                      transition: 'transform 0.2s ease',
                      backdropFilter: msg.type === 'ai' ? 'blur(10px)' : 'none'
                    }}
                    className="message-bubble"
                  >
                    {msg.type === 'ai' && (
                      <>
                        {/* Decorative corner accent */}
                        <div style={{
                          position: 'absolute',
                          top: 0,
                          left: 0,
                          width: '50px',
                          height: '50px',
                          background: 'radial-gradient(circle at top left, rgba(21, 128, 61, 0.08) 0%, transparent 70%)',
                          pointerEvents: 'none'
                        }}></div>
                        <div style={{
                          display: 'flex',
                          alignItems: 'center',
                          gap: '10px',
                          marginBottom: '12px',
                          paddingBottom: '12px',
                          borderBottom: '1px solid rgba(21, 128, 61, 0.12)'
                        }}>
                          <div style={{
                            width: '10px',
                            height: '10px',
                            background: 'linear-gradient(135deg, #22c55e 0%, #16a34a 100%)',
                            borderRadius: '50%',
                            boxShadow: '0 0 10px rgba(34, 197, 94, 0.5)',
                            animation: 'statusBlink 2s ease-in-out infinite'
                          }}></div>
                          <span style={{
                            fontSize: '11px',
                            fontWeight: '700',
                            color: '#15803d',
                            textTransform: 'uppercase',
                            letterSpacing: '1px'
                          }}>
                            AI Assistant
                          </span>
                        </div>
                      </>
                    )}
                    <p style={{
                      fontSize: '14.5px',
                      lineHeight: '1.7',
                      margin: 0,
                      color: msg.type === 'ai' ? 'var(--hb-black-text)' : 'white',
                      whiteSpace: 'pre-wrap',
                      wordBreak: 'break-word',
                      position: 'relative',
                      zIndex: 1
                    }}>
                      {msg.text}
                    </p>

                    {/* Citations */}
                    {msg.sources && msg.sources.length > 0 && (
                      <div style={{
                        marginTop: '16px',
                        paddingTop: '16px',
                        borderTop: '1px solid rgba(21, 128, 61, 0.12)'
                      }}>
                        <p style={{
                          fontSize: '11px',
                          fontWeight: '700',
                          color: '#15803d',
                          marginBottom: '12px',
                          textTransform: 'uppercase',
                          letterSpacing: '1px',
                          display: 'flex',
                          alignItems: 'center',
                          gap: '6px'
                        }}>
                          <span style={{
                            display: 'inline-block',
                            width: '4px',
                            height: '4px',
                            borderRadius: '50%',
                            background: '#15803d'
                          }}></span>
                          📚 Sources
                        </p>
                        <div style={{ display: 'flex', flexDirection: 'column', gap: '10px' }}>
                          {msg.sources.map((source, idx) => (
                            <a
                              key={idx}
                              href={source.url}
                              target="_blank"
                              rel="noopener noreferrer"
                              className="source-link"
                              style={{
                                display: 'flex',
                                alignItems: 'flex-start',
                                gap: '10px',
                                fontSize: '12.5px',
                                color: '#15803d',
                                textDecoration: 'none',
                                padding: '12px',
                                borderRadius: '12px',
                                background: 'rgba(21, 128, 61, 0.05)',
                                border: '1px solid rgba(21, 128, 61, 0.1)',
                                transition: 'all 0.3s ease',
                                position: 'relative',
                                overflow: 'hidden'
                              }}
                            >
                              <ExternalLink style={{ width: '16px', height: '16px', marginTop: '2px', flexShrink: 0 }} />
                              <span>
                                <span style={{ fontWeight: '700', display: 'block', marginBottom: '4px' }}>{source.title}</span>
                                {source.snippet && (
                                  <span style={{ color: 'var(--text-color)', fontSize: '11.5px', display: 'block' }}>
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
                    color: 'var(--text-color)',
                    marginTop: '8px',
                    paddingLeft: msg.type === 'user' ? '0' : '10px',
                    paddingRight: msg.type === 'user' ? '10px' : '0',
                    fontWeight: '500'
                  }}>
                    {msg.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
                  </span>
                </div>
              </div>
            ))}

            {isTyping && (
              <div style={{
                display: 'flex',
                gap: '14px',
                animation: 'slideIn 0.4s cubic-bezier(0.4, 0, 0.2, 1)',
                position: 'relative',
                zIndex: 1
              }}>
                <div style={{
                  width: '40px',
                  height: '40px',
                  borderRadius: '50%',
                  background: 'linear-gradient(135deg, #15803d 0%, #16a34a 100%)',
                  display: 'flex',
                  alignItems: 'center',
                  justifyContent: 'center',
                  boxShadow: '0 8px 20px rgba(21, 128, 61, 0.35)',
                  border: '2px solid rgba(255, 255, 255, 0.2)'
                }}>
                  <Loader2 style={{ width: '22px', height: '22px', color: 'white', animation: 'spin 1s linear infinite' }} />
                </div>
                <div style={{
                  backgroundColor: 'var(--hb-black)',
                  border: '1px solid rgba(21, 128, 61, 0.15)',
                  borderRadius: '20px 20px 20px 4px',
                  padding: '16px 20px',
                  boxShadow: '0 4px 15px rgba(21, 128, 61, 0.12)',
                  backdropFilter: 'blur(10px)'
                }}>
                  <div style={{ display: 'flex', gap: '8px', alignItems: 'center' }}>
                    <div style={{
                      width: '10px',
                      height: '10px',
                      background: 'linear-gradient(135deg, #22c55e 0%, #16a34a 100%)',
                      borderRadius: '50%',
                      boxShadow: '0 0 10px rgba(34, 197, 94, 0.5)',
                      animation: 'bounce 1.4s ease-in-out 0s infinite'
                    }}></div>
                    <div style={{
                      width: '10px',
                      height: '10px',
                      background: 'linear-gradient(135deg, #22c55e 0%, #16a34a 100%)',
                      borderRadius: '50%',
                      boxShadow: '0 0 10px rgba(34, 197, 94, 0.5)',
                      animation: 'bounce 1.4s ease-in-out 0.2s infinite'
                    }}></div>
                    <div style={{
                      width: '10px',
                      height: '10px',
                      background: 'linear-gradient(135deg, #22c55e 0%, #16a34a 100%)',
                      borderRadius: '50%',
                      boxShadow: '0 0 10px rgba(34, 197, 94, 0.5)',
                      animation: 'bounce 1.4s ease-in-out 0.4s infinite'
                    }}></div>
                    <span style={{ fontSize: '13px', color: 'var(--text-color)', marginLeft: '10px', fontWeight: '600' }}>
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
            padding: '20px 24px',
            backgroundColor: 'var(--hb-black)',
            borderTop: '1px solid rgba(21, 128, 61, 0.15)',
            borderBottomLeftRadius: '24px',
            borderBottomRightRadius: '24px'
          }}>
            {selectedTextContext && (
              <div style={{
                background: 'linear-gradient(135deg, rgba(21, 128, 61, 0.08) 0%, rgba(22, 163, 74, 0.08) 100%)',
                padding: '12px 16px',
                borderRadius: '14px',
                marginBottom: '14px',
                fontSize: '13px',
                color: '#15803d',
                display: 'flex',
                alignItems: 'center',
                gap: '10px',
                border: '1px solid rgba(21, 128, 61, 0.15)',
                boxShadow: '0 2px 8px rgba(21, 128, 61, 0.08)'
              }}>
                <Sparkles style={{ width: '16px', height: '16px', flexShrink: 0 }} />
                <span style={{ flex: 1, fontWeight: '600' }}>
                  Selected text will be used as context
                </span>
                <button
                  onClick={() => setSelectedTextContext(null)}
                  className="context-close-button"
                  style={{
                    background: 'rgba(21, 128, 61, 0.1)',
                    border: 'none',
                    borderRadius: '8px',
                    cursor: 'pointer',
                    padding: '6px',
                    display: 'flex',
                    alignItems: 'center',
                    color: '#15803d',
                    transition: 'all 0.2s'
                  }}
                >
                  <X style={{ width: '16px', height: '16px' }} />
                </button>
              </div>
            )}
            <div style={{ display: 'flex', gap: '12px' }}>
              <input
                type="text"
                value={input}
                onChange={(e) => setInput(e.target.value)}
                onKeyPress={handleKeyPress}
                placeholder="Ask about robotics, AI, simulation..."
                disabled={isTyping}
                className="chat-input"
                style={{
                  flex: 1,
                  padding: '16px 18px',
                  backgroundColor: 'var(--hb-black)',
                  color: 'var(--hb-black-text)',
                  borderRadius: '16px',
                  border: '2px solid rgba(21, 128, 61, 0.15)',
                  fontSize: '14.5px',
                  outline: 'none',
                  transition: 'all 0.3s cubic-bezier(0.4, 0, 0.2, 1)',
                  opacity: isTyping ? 0.6 : 1,
                  fontWeight: '500',
                  boxShadow: '0 2px 8px rgba(21, 128, 61, 0.05)'
                }}
              />
              <button
                onClick={() => handleSend(null, !!selectedTextContext)}
                disabled={!input.trim() || isTyping}
                className="send-button"
                style={{
                  background: 'linear-gradient(135deg, #15803d 0%, #16a34a 100%)',
                  color: 'white',
                  padding: '16px',
                  borderRadius: '16px',
                  border: 'none',
                  cursor: isTyping || !input.trim() ? 'not-allowed' : 'pointer',
                  display: 'flex',
                  alignItems: 'center',
                  justifyContent: 'center',
                  transition: 'all 0.3s cubic-bezier(0.4, 0, 0.2, 1)',
                  opacity: isTyping || !input.trim() ? 0.5 : 1,
                  boxShadow: isTyping || !input.trim() ? 'none' : '0 8px 20px rgba(21, 128, 61, 0.35)',
                  minWidth: '56px',
                  position: 'relative',
                  overflow: 'hidden'
                }}
              >
                <div style={{
                  position: 'absolute',
                  inset: 0,
                  background: 'radial-gradient(circle at center, rgba(255, 255, 255, 0.2) 0%, transparent 70%)',
                  opacity: 0,
                  transition: 'opacity 0.3s',
                  pointerEvents: 'none'
                }} className="button-glow"></div>
                {isTyping ? (
                  <Loader2 style={{ width: '22px', height: '22px', animation: 'spin 1s linear infinite' }} />
                ) : (
                  <Send style={{ width: '22px', height: '22px' }} />
                )}
              </button>
            </div>
            <p style={{
              fontSize: '11px',
              color: 'var(--text-color)',
              textAlign: 'center',
              marginTop: '12px',
              marginBottom: 0,
              fontWeight: '600',
              letterSpacing: '0.3px'
            }}>
              Powered by Cohere + Qdrant + Gemini
            </p>
          </div>
        </div>

        {/* Floating Button */}
        <button
          onClick={() => setIsOpen(!isOpen)}
          className="floating-button"
          style={{
            width: '70px',
            height: '70px',
            background: 'linear-gradient(135deg, #15803d 0%, #16a34a 100%)',
            borderRadius: '50%',
            border: '3px solid rgba(255, 255, 255, 0.2)',
            boxShadow: '0 12px 35px rgba(21, 128, 61, 0.45), 0 0 0 0 rgba(21, 128, 61, 0.4)',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center',
            cursor: 'pointer',
            transition: 'all 0.4s cubic-bezier(0.68, -0.55, 0.265, 1.55)',
            animation: 'float 3s ease-in-out infinite, ripple 2s ease-out infinite',
            position: 'relative',
            zIndex: 10001
          }}
        >
          <div style={{
            position: 'absolute',
            inset: -3,
            borderRadius: '50%',
            background: 'linear-gradient(135deg, rgba(21, 128, 61, 0.3), rgba(22, 163, 74, 0.3))',
            animation: 'rotate 4s linear infinite',
            filter: 'blur(8px)'
          }}></div>
          {isOpen ? (
            <X style={{ width: '34px', height: '34px', color: 'white', position: 'relative', zIndex: 1 }} />
          ) : (
            <>
              <MessageCircle style={{ width: '34px', height: '34px', color: 'white', position: 'relative', zIndex: 1 }} />
              <div style={{
                position: 'absolute',
                top: '8px',
                right: '8px',
                width: '18px',
                height: '18px',
                background: 'linear-gradient(135deg, #ef4444 0%, #dc2626 100%)',
                borderRadius: '50%',
                animation: 'statusBlink 2s ease-in-out infinite',
                border: '3px solid white',
                boxShadow: '0 2px 8px rgba(239, 68, 68, 0.5)',
                zIndex: 2
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
              transform: translateY(-12px);
            }
          }

          @keyframes slideIn {
            from {
              opacity: 0;
              transform: translateY(15px) scale(0.95);
            }
            to {
              opacity: 1;
              transform: translateY(0) scale(1);
            }
          }

          @keyframes statusBlink {
            0%, 100% {
              opacity: 1;
              transform: scale(1);
            }
            50% {
              opacity: 0.6;
              transform: scale(1.15);
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
              transform: translateY(-8px);
            }
          }

          @keyframes shimmer {
            0%, 100% {
              opacity: 0.4;
              transform: translateX(-50%);
            }
            50% {
              opacity: 0.8;
              transform: translateX(50%);
            }
          }

          @keyframes sparkle {
            0%, 100% {
              opacity: 1;
              transform: scale(1) rotate(0deg);
            }
            50% {
              opacity: 0.5;
              transform: scale(1.2) rotate(180deg);
            }
          }

          @keyframes avatarPulse {
            0%, 100% {
              transform: scale(1);
              box-shadow: 0 8px 20px rgba(0, 0, 0, 0.2), 0 0 0 4px rgba(255, 255, 255, 0.2);
            }
            50% {
              transform: scale(1.05);
              box-shadow: 0 12px 25px rgba(0, 0, 0, 0.25), 0 0 0 6px rgba(255, 255, 255, 0.3);
            }
          }

          @keyframes avatarBounce {
            0% {
              opacity: 0;
              transform: scale(0.8) translateY(10px);
            }
            60% {
              transform: scale(1.1) translateY(-5px);
            }
            100% {
              opacity: 1;
              transform: scale(1) translateY(0);
            }
          }

          @keyframes ripple {
            0% {
              box-shadow: 0 12px 35px rgba(21, 128, 61, 0.45), 0 0 0 0 rgba(21, 128, 61, 0.4);
            }
            50% {
              box-shadow: 0 12px 35px rgba(21, 128, 61, 0.45), 0 0 0 15px rgba(21, 128, 61, 0);
            }
            100% {
              box-shadow: 0 12px 35px rgba(21, 128, 61, 0.45), 0 0 0 0 rgba(21, 128, 61, 0);
            }
          }

          @keyframes rotate {
            from {
              transform: rotate(0deg);
            }
            to {
              transform: rotate(360deg);
            }
          }

          .chatbot-window::-webkit-scrollbar {
            width: 8px;
          }

          .chatbot-window::-webkit-scrollbar-track {
            background: rgba(21, 128, 61, 0.05);
            border-radius: 10px;
          }

          .chatbot-window::-webkit-scrollbar-thumb {
            background: linear-gradient(to bottom, #15803d, #16a34a);
            border-radius: 10px;
            border: 2px solid transparent;
            background-clip: padding-box;
          }

          .chatbot-window::-webkit-scrollbar-thumb:hover {
            background: linear-gradient(to bottom, #14532d, #15803d);
            background-clip: padding-box;
          }

          .header-button:hover {
            background: rgba(255, 255, 255, 0.3) !important;
            transform: scale(1.1) rotate(5deg);
            box-shadow: 0 4px 12px rgba(0, 0, 0, 0.2);
          }

          .header-button:active {
            transform: scale(0.95) rotate(0deg);
          }

          .message-bubble:hover {
            transform: translateY(-2px);
          }

          .source-link:hover {
            background: rgba(21, 128, 61, 0.12) !important;
            border-color: rgba(21, 128, 61, 0.25) !important;
            transform: translateX(4px);
            box-shadow: 0 4px 12px rgba(21, 128, 61, 0.15);
          }

          .chat-input:focus {
            border-color: #15803d !important;
            box-shadow: 0 0 0 4px rgba(21, 128, 61, 0.1), 0 4px 12px rgba(21, 128, 61, 0.15) !important;
            transform: scale(1.01);
          }

          .send-button:not(:disabled):hover {
            transform: scale(1.08) rotate(5deg);
            box-shadow: 0 12px 28px rgba(21, 128, 61, 0.45) !important;
          }

          .send-button:not(:disabled):hover .button-glow {
            opacity: 1;
          }

          .send-button:not(:disabled):active {
            transform: scale(0.95) rotate(0deg);
          }

          .floating-button:hover {
            transform: scale(1.15) rotate(10deg);
            box-shadow: 0 16px 45px rgba(21, 128, 61, 0.55), 0 0 0 0 rgba(21, 128, 61, 0.4);
          }

          .floating-button:active {
            transform: scale(1.05) rotate(0deg);
          }

          .context-close-button:hover {
            background: rgba(21, 128, 61, 0.2) !important;
            transform: rotate(90deg);
          }
        `}</style>
      </div>
    </>
  );
};

export default HumanoidChatbot;