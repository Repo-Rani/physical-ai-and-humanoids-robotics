export default function Features() {
    const featuresData = [
        {
            icon: <svg className='text-white' xmlns="http://www.w3.org/2000/svg" width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"><path d="M20 13c0 5-3.5 7.5-7.66 8.95a1 1 0 0 1-.67-.01C7.5 20.5 4 18 4 13V6a1 1 0 0 1 1-1c2 0 4.5-1.2 6.24-2.72a1.17 1.17 0 0 1 1.52 0C14.51 3.81 17 5 19 5a1 1 0 0 1 1 1z" /><path d="m9 12 2 2 4-4" /></svg>,
            title: "Task Automation",
            description: "Let AI handle the repetitive, time-consuming tasks so your team can stay focused on business growth."
        },
        {
            icon: <svg className='text-white' xmlns="http://www.w3.org/2000/svg" width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"><path d="M3 12a9 9 0 1 0 9-9 9.75 9.75 0 0 0-6.74 2.74L3 8" /><path d="M3 3v5h5" /><path d="M12 7v5l4 2" /></svg>,
            title: "Real-Time Monitoring",
            description: "Empower your business by letting AI take over repetitive tasks and freeing your team for high impact work."
        },
        {
            icon: <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg"><path d="M21 7.9999C20.9996 7.64918 20.9071 7.30471 20.7315 7.00106C20.556 6.69742 20.3037 6.44526 20 6.2699L13 2.2699C12.696 2.09437 12.3511 2.00195 12 2.00195C11.6489 2.00195 11.304 2.09437 11 2.2699L4 6.2699C3.69626 6.44526 3.44398 6.69742 3.26846 7.00106C3.09294 7.30471 3.00036 7.64918 3 7.9999V15.9999C3.00036 16.3506 3.09294 16.6951 3.26846 16.9987C3.44398 17.3024 3.69626 17.5545 4 17.7299L11 21.7299C11.304 21.9054 11.6489 21.9979 12 21.9979C12.3511 21.9979 12.696 21.9054 13 21.7299L20 17.7299C20.3037 17.5545 20.556 17.3024 20.7315 16.9987C20.9071 16.6951 20.9996 16.3506 21 15.9999V7.9999Z" stroke="white" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round" /><path d="M3.29999 7L12 12L20.7 7" stroke="white" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round" /><path d="M12 22V12" stroke="white" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round" /></svg>,
            title: "Context Awareness",
            description: "AI takes care of the repetitive stuff, so your team can focus on growth and delivering results that matter."
        },
        {
            icon: <svg className='text-white' xmlns="http://www.w3.org/2000/svg" width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"><ellipse cx="12" cy="5" rx="9" ry="3" /><path d="M3 5V19A9 3 0 0 0 21 19V5" /><path d="M3 12A9 3 0 0 0 21 12" /></svg>,
            title: "Resource Optimization",
            description: "Empower your business by letting AI take over repetitive tasks and freeing team for high impact work."
        },
        {
            icon: <svg className='text-white' xmlns="http://www.w3.org/2000/svg" width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"><path d="M16 21v-2a4 4 0 0 0-4-4H6a4 4 0 0 0-4 4v2" /><path d="M16 3.128a4 4 0 0 1 0 7.744" /><path d="M22 21v-2a4 4 0 0 0-3-3.87" /><circle cx="9" cy="7" r="4" /></svg>,
            title: "Role-Based Access",
            description: "Free your team from manual, repetitive work. Let AI automate the busywork while you focus on scaling."
        },
        {
            icon: <svg className='text-white' xmlns="http://www.w3.org/2000/svg" width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"><path d="M5 17H4a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h16a2 2 0 0 1 2 2v10a2 2 0 0 1-2 2h-1" /><path d="m12 15 5 6H7Z" /></svg>,
            title: "AI-Agent Collaboration",
            description: "Let AI handle the repetitive, time-consuming tasks so your team can stay focused on business growth."
        }
    ];

    return (
        <>
            <style>
                {`@import url("https://fonts.googleapis.com/css2?family=Poppins:ital,wght@0,100;0,200;0,300;0,400;0,500;0,600;0,700;0,800;0,900;1,100;1,200;1,300;1,400;1,500;1,600;1,700;1,800;1,900&display=swap");

                * {
                    font-family: "Poppins", sans-serif;
                }

                @keyframes fadeInUp {
                    from {
                        opacity: 0;
                        transform: translateY(30px);
                    }
                    to {
                        opacity: 1;
                        transform: translateY(0);
                    }
                }

                @keyframes slideInFromLeft {
                    from {
                        opacity: 0;
                        transform: translateX(-50px);
                    }
                    to {
                        opacity: 1;
                        transform: translateX(0);
                    }
                }

                @keyframes pulse {
                    0%, 100% {
                        opacity: 1;
                    }
                    50% {
                        opacity: 0.8;
                    }
                }

                @keyframes float {
                    0%, 100% {
                        transform: translateY(0px);
                    }
                    50% {
                        transform: translateY(-10px);
                    }
                }

                @keyframes scaleIn {
                    from {
                        opacity: 0;
                        transform: scale(0.9);
                    }
                    to {
                        opacity: 1;
                        transform: scale(1);
                    }
                }

                @keyframes shimmer {
                    0% {
                        background-position: -1000px 0;
                    }
                    100% {
                        background-position: 1000px 0;
                    }
                }

                @keyframes shimmerOnce {
                    0% {
                        background-position: -1000px 0;
                    }
                    100% {
                        background-position: 1000px 0;
                    }
                }

                .animate-fade-in-up {
                    animation: fadeInUp 0.8s ease-out forwards;
                }

                .animate-slide-in {
                    animation: slideInFromLeft 0.8s ease-out forwards;
                }

                .animate-scale-in {
                    animation: scaleIn 0.6s ease-out forwards;
                }

                .feature-card {
                    position: relative;
                    overflow: hidden;
                    transition: all 0.4s cubic-bezier(0.4, 0, 0.2, 1);
                }

                .feature-card::before {
                    content: '';
                    position: absolute;
                    top: 0;
                    left: -100%;
                    width: 100%;
                    height: 100%;
                    background: linear-gradient(90deg, transparent, rgba(21, 128, 61, 0.1), transparent);
                    transition: left 0.6s;
                }

                .feature-card:hover::before {
                    left: 100%;
                }

                .feature-card:hover {
                    transform: translateY(-8px) scale(1.02);
                    box-shadow: 0 20px 40px rgba(21, 128, 61, 0.2), 0 0 20px rgba(22, 163, 74, 0.15);
                    border-color: rgba(21, 128, 61, 0.5);
                }

                .icon-wrapper {
                    transition: all 0.4s cubic-bezier(0.4, 0, 0.2, 1);
                }

                .feature-card:hover .icon-wrapper {
                    transform: scale(1.15) rotate(5deg);
                }

                .badge-button {
                    position: relative;
                    overflow: hidden;
                    transition: all 0.3s ease;
                }

                .badge-button::before {
                    content: '';
                    position: absolute;
                    top: 50%;
                    left: 50%;
                    width: 0;
                    height: 0;
                    border-radius: 50%;
                    background: rgba(21, 128, 61, 0.2);
                    transform: translate(-50%, -50%);
                    transition: width 0.4s, height 0.4s;
                }

                .badge-button:hover::before {
                    width: 300px;
                    height: 300px;
                }

                .badge-button:hover {
                    border-color: #15803d;
                    color: #15803d;
                    transform: translateY(-2px);
                    box-shadow: 0 4px 12px rgba(21, 128, 61, 0.2);
                }

                .heading-gradient {
                    background: linear-gradient(135deg, #ffffff 0%, #d1fae5 50%, #ffffff 100%);
                    background-size: 200% auto;
                    -webkit-background-clip: text;
                    -webkit-text-fill-color: transparent;
                    background-clip: text;
                    animation: shimmerOnce 5s linear forwards;
                    animation-iteration-count: 1;
                }

                .stagger-1 { animation-delay: 0.1s; }
                .stagger-2 { animation-delay: 0.2s; }
                .stagger-3 { animation-delay: 0.3s; }
                .stagger-4 { animation-delay: 0.4s; }
                .stagger-5 { animation-delay: 0.5s; }
                .stagger-6 { animation-delay: 0.6s; }
                `}
            </style>

            <section className="py-20 px-4 bg-black flex flex-col justify-center items-center gap-6 bg-[url('https://raw.githubusercontent.com/prebuiltui/prebuiltui/main/assets/hero/green-gradient-bg.svg')] bg-top bg-no-repeat relative overflow-hidden">
                {/* Decorative Background Elements */}
                <div className="absolute inset-0 pointer-events-none">
                    <div className="absolute top-1/4 left-1/4 w-96 h-96 bg-green-600/5 rounded-full blur-3xl"></div>
                    <div className="absolute bottom-1/4 right-1/4 w-96 h-96 bg-green-500/5 rounded-full blur-3xl"></div>
                </div>

                {/* Badge */}
                <button className='badge-button px-6 h-10 border border-gray-700 text-slate-200 text-sm font-semibold rounded-xl relative z-10 animate-fade-in-up backdrop-blur-sm bg-black/20'>
                    ✨ Features
                </button>

                {/* Heading */}
                <h2 className="heading-gradient text-3xl md:text-[42px]/tight font-bold max-w-2xl text-center leading-tight animate-fade-in-up stagger-1 relative z-10">
                    AI Agents That Automate and Accelerate Growth
                </h2>

                {/* Description */}
                <p className='text-base/7 text-gray-300 max-w-2xl text-center font-medium animate-fade-in-up stagger-2 relative z-10'>
                    Streamline operations, boost productivity, and scale effortlessly - all powered by intelligent automation.
                </p>

                {/* Features Grid */}
                <div className="relative max-w-6xl mx-auto grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6 mt-8 z-10">
                    {featuresData.map((feature, index) => (
                        <div 
                            key={index} 
                            className={`feature-card bg-gradient-to-br from-[#0a0a0a] via-[#0d1117] to-[#0a0a0a] border-2 border-gray-800/50 rounded-2xl p-8 space-y-4 animate-scale-in stagger-${index + 1} backdrop-blur-sm`}
                            style={{
                                animationDelay: `${index * 0.1}s`,
                                opacity: 0,
                                animationFillMode: 'forwards'
                            }}
                        >
                            {/* Icon Container */}
                            <div className="icon-wrapper w-14 h-14 rounded-xl bg-gradient-to-br from-green-600 to-green-700 flex items-center justify-center shadow-lg shadow-green-600/30">
                                {feature.icon}
                            </div>

                            {/* Title */}
                            <h3 className='font-bold text-xl text-gray-100 tracking-tight'>
                                {feature.title}
                            </h3>

                            {/* Description */}
                            <p className='text-sm/6 text-gray-400 font-medium'>
                                {feature.description}
                            </p>

                            {/* Decorative Bottom Line */}
                            <div className="pt-3">
                                <div className="h-1 w-16 bg-gradient-to-r from-green-600 to-green-400 rounded-full"></div>
                            </div>
                        </div>
                    ))}
                </div>

                {/* Bottom Decorative Element */}
                <div className="mt-12 relative z-10 animate-fade-in-up stagger-6">
                    <div className="flex items-center gap-2 text-sm text-gray-500">
                        <div className="w-2 h-2 bg-green-600 rounded-full animate-pulse"></div>
                        <span className="font-semibold">Powered by Advanced AI Technology</span>
                        <div className="w-2 h-2 bg-green-600 rounded-full animate-pulse"></div>
                    </div>
                </div>
            </section>
        </>
    )
}