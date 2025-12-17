
export default function PhysicalAIParadigmShift() {
  return (
    <section className="py-14 bg-[var(--color-bg)]">
      <div className="container mx-auto px-4">
       

       

        {/* Comparison Cards */}
        <div className="grid grid-cols-1 lg:grid-cols-2 gap-12 max-w-6xl mx-auto relative">
          {/* Traditional Approach */}
          <div className="group bg-[var(--color-bg)] border border-gray-200 rounded-xl p-8 shadow-sm hover:shadow-xl transition-all duration-300 animate-fadeIn delay-400 relative overflow-hidden">

            <div className="absolute inset-0 bg-gradient-to-br from-gray-50/0 to-gray-50/0 group-hover:from-[var(--color-physical-from-shift-hover-shadow-effect)] group-hover:to-[var(--color-physical-to-shift-hover-shadow-effect)] transition-all duration-300"></div>
            
            <div className="relative z-10">
              {/* Icon and Title */}
              <div className="flex items-center gap-3 mb-6">
                <div className="flex items-center justify-center w-12 h-12 bg-gray-100 rounded-lg group-hover:scale-110 transition-transform duration-300">
                  <svg className="w-6 h-6 text-gray-600" fill="none" stroke="text-green-500" viewBox="0 0 24 24">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 6.253v13m0-13C10.832 5.477 9.246 5 7.5 5S4.168 5.477 3 6.253v13C4.168 18.477 5.754 18 7.5 18s3.332.477 4.5 1.253m0-13C13.168 5.477 14.754 5 16.5 5c1.747 0 3.332.477 4.5 1.253v13C19.832 18.477 18.247 18 16.5 18c-1.746 0-3.332.477-4.5 1.253" />
                  </svg>
                </div>
                <div>
                  <h3 className="text-xl font-bold text-[var(--color-card-description-text)]">Traditional Robotics</h3>
                  <p className="text-[12px] text-gray-500 italic">The pre-programmed era</p>
                </div>
              </div>

              {/* Feature List */}
              <div className="space-y-4">
                <div className="flex gap-3 group/item">
                  <div className="flex-shrink-0 w-1.5 h-1.5 bg-gray-400 rounded-full mt-2"></div>
                  <div>
                    <p className="font-semibold text-[var(--color-card-description-text)] text-[15px] mb-1">Instruction-Based Control</p>
                    <p className="text-[var(--color-description-text)] text-[13px]">Program robots with exact trajectories and fixed behavior trees</p>
                  </div>
                </div>

                <div className="flex gap-3 group/item">
                  <div className="flex-shrink-0 w-1.5 h-1.5 bg-gray-400 rounded-full mt-2"></div>
                  <div>
                    <p className="font-semibold text-[var(--color-card-description-text)] text-[15px] mb-1">Manual Calibration</p>
                    <p className="text-[var(--color-description-text)] text-[13px]">Engineer hand-tunes sensors, actuators, and kinematics</p>
                  </div>
                </div>

                <div className="flex gap-3 group/item">
                  <div className="flex-shrink-0 w-1.5 h-1.5 bg-gray-400 rounded-full mt-2"></div>
                  <div>
                    <p className="font-semibold text-[var(--color-card-description-text)] text-[15px] mb-1">Static Environments</p>
                    <p className="text-[var(--color-description-text)] text-[13px]">Robots operate in controlled factory settings with predictable conditions</p>
                  </div>
                </div>

                <div className="flex gap-3 group/item">
                  <div className="flex-shrink-0 w-1.5 h-1.5 bg-gray-400 rounded-full mt-2"></div>
                  <div>
                    <p className="font-semibold text-[var(--color-card-description-text)] text-[15px] mb-1">Linear Learning Path</p>
                    <p className="text-[var(--color-description-text)] text-[13px]">Learn mechanics → Study control theory → Build simple bots → Slowly scale</p>
                  </div>
                </div>

                <div className="flex gap-3 group/item">
                  <div className="flex-shrink-0 w-1.5 h-1.5 bg-gray-400 rounded-full mt-2"></div>
                  <div>
                    <p className="font-semibold text-[var(--color-card-description-text)] text-[15px] mb-1">Hardware-First Approach</p>
                    <p className="text-[var(--color-description-text)] text-[13px]">Focus on mechanical design and motor control from day one</p>
                  </div>
                </div>
              </div>
            </div>
          </div>

          {/* Physical AI Approach */}
          <div className="group bg-gradient-to-br from-[var(--color-great-shift-from-shadow-color)] to-[var(--color-great-shift-to-shadow-color)] border-2 border-orange-200 rounded-xl p-8 shadow-lg hover:shadow-2xl transition-all duration-300 animate-fadeIn delay-500 relative overflow-hidden">
            {/* Premium Badge */}
            <div className="absolute top-4 right-4 bg-gradient-to-r from-green-500 to-green-600 text-white text-[10px] font-bold px-3 py-1 rounded-full shadow-md z-20">
              FUTURE OF ROBOTICS
            </div>

            <div className="absolute inset-0 bg-gradient-to-br from-orange-50/0 to-orange-100/0 group-hover:from-orange-50/30 group-hover:to-orange-100/20 transition-all duration-300"></div>
            
            <div className="relative z-10">
              {/* Icon and Title */}
              <div className="flex items-center gap-3 mb-6">
                <div className="flex items-center justify-center w-12 h-12 bg-orange-100 rounded-lg group-hover:scale-110 transition-transform duration-300">
                  <svg className="w-6 h-6 text-[var(--ifm-color-primary)]" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M13 10V3L4 14h7v7l9-11h-7z" />
                  </svg>
                </div>
                <div>
                  <h3 className="text-xl font-bold text-[var(--ifm-color-primary)]">Physical AI Way</h3>
                  <p className="text-[12px] text-[var(--ifm-color-primary)] italic">The intelligence era</p>
                </div>
              </div>

              {/* Feature List */}
              <div className="space-y-4">
                <div className="flex gap-3 group/item">
                  <div className="flex-shrink-0 w-1.5 h-1.5 bg-white rounded-full mt-2"></div>
                  <div>
                    <p className="font-semibold text-[var(--ifm-color-primary)] text-[15px] mb-1">Intent-Based Intelligence</p>
                    <p className="text-gray-700 text-[13px]">Describe goals; AI reasons physics, plans motion, and adapts behavior</p>
                  </div>
                </div>

                <div className="flex gap-3 group/item">
                  <div className="flex-shrink-0 w-1.5 h-1.5 bg-white rounded-full mt-2"></div>
                  <div>
                    <p className="font-semibold text-[var(--ifm-color-primary)] text-[15px] mb-1">Self-Learning Systems</p>
                    <p className="text-gray-700 text-[13px]">You and AI collaborate—robot improves through reinforcement and simulation</p>
                  </div>
                </div>

                <div className="flex gap-3 group/item">
                  <div className="flex-shrink-0 w-1.5 h-1.5 bg-white rounded-full mt-2"></div>
                  <div>
                    <p className="font-semibold text-[var(--ifm-color-primary)] text-[15px] mb-1">Digital Twin Simulation</p>
                    <p className="text-gray-700 text-[13px]">Train in virtual worlds (Isaac Sim), deploy to real humanoids seamlessly</p>
                  </div>
                </div>

                <div className="flex gap-3 group/item">
                  <div className="flex-shrink-0 w-1.5 h-1.5 bg-white rounded-full mt-2"></div>
                  <div>
                    <p className="font-semibold text-[var(--ifm-color-primary)] text-[15px] mb-1">Production-First Learning</p>
                    <p className="text-gray-700 text-[13px]">Build autonomous humanoids from day one using ROS 2 and NVIDIA Isaac</p>
                  </div>
                </div>

                <div className="flex gap-3 group/item">
                  <div className="flex-shrink-0 w-1.5 h-1.5 bg-white rounded-full mt-2"></div>
                  <div>
                    <p className="font-semibold text-[var(--ifm-color-primary)] text-[15px] mb-1">Architecture-First Design</p>
                    <p className="text-gray-700 text-[13px]">Design intelligent perception pipelines, not just joint controllers</p>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </div>

        {/* VS Divider - Desktop only */}
        <div className="hidden lg:flex relative left-1/2 top-[-520px] -translate-x-1/2 -translate-y-1/2 items-center justify-center w-16 h-16 bg-white border-4 border-orange-200 rounded-full shadow-lg z-10">
          <span className="text-[var(--ifm-color-primary)] font-bold text-sm">VS</span>
        </div>

        {/* Bottom CTA */}
        <div className="mt-4 text-center animate-fadeIn delay-600 shadow-sm hover:shadow-lg transition-all duration-300">
        </div>
      </div>

      <style>{`
        @keyframes fadeIn {
          from { opacity: 0; transform: translateY(10px); }
          to { opacity: 1; transform: translateY(0); }
        }
        @keyframes slideUp {
          from { opacity: 0; transform: translateY(25px); }
          to { opacity: 1; transform: translateY(0); }
        }
        .animate-fadeIn { 
          animation: fadeIn 0.6s ease forwards; 
          opacity: 0;
        }
        .animate-slideUp { 
          animation: slideUp 0.7s ease forwards; 
          opacity: 0;
        }
        .delay-200 { animation-delay: 0.2s; }
        .delay-300 { animation-delay: 0.3s; }
        .delay-400 { animation-delay: 0.4s; }
        .delay-500 { animation-delay: 0.5s; }
        .delay-600 { animation-delay: 0.6s; }
      `}</style>
    </section>
  );
}